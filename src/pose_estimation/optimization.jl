"""
Pose estimation optimization using nonlinear least squares.

This module implements pose estimation by minimizing reprojection errors
using SimpleNonlinearSolve.jl and integrating with ProbabilisticParameterEstimators
noise models.
"""

using SimpleNonlinearSolve
using Moshi: @match
import NonlinearSolveFirstOrder: LevenbergMarquardtTrustRegion, LevenbergMarquardt
import SciMLBase: successful_retcode
using StaticArrays
using LinearAlgebra
using Rotations

"""
    PoseOptimizationParams6DOF{T, T′, S, RC, OC, M}

Parameters for 6-DOF pose optimization (position + attitude).
"""
struct PoseOptimizationParams6DOF{T, T′, S, RC<:AbstractVector{WorldPoint{T}}, OC<:AbstractVector{ProjectionPoint{T′,S}}, M<:UpperTriangular{Float64, <:AbstractMatrix{Float64}}}
    runway_corners::RC
    observed_corners::OC
    config::CameraConfig{S}
    chol_upper::M

    function PoseOptimizationParams6DOF(
            runway_corners::RC,
            observed_corners::OC,
            config::CameraConfig{S},
            noise_model
        ) where {T, T′, S, RC<:AbstractVector{WorldPoint{T}}, OC<:AbstractVector{ProjectionPoint{T′,S}}}
        Σ = covmatrix(noise_model)
        F = cholesky(Σ) # L is not used
        U = F.U
        return new{T, T′, S, RC, OC, typeof(U)}(runway_corners, observed_corners, config, U)
    end
end

"""
    PoseOptimizationParams3DOF{T, T′, S, A, RC, OC, M}

Parameters for 3-DOF pose optimization (position only with known attitude).
"""
struct PoseOptimizationParams3DOF{T, T′, S, A<:Rotation{3}, RC<:AbstractVector{WorldPoint{T}}, OC<:AbstractVector{ProjectionPoint{T′,S}}, M<:UpperTriangular{Float64, <:AbstractMatrix{Float64}}}
    runway_corners::RC
    observed_corners::OC
    config::CameraConfig{S}
    chol_upper::M
    known_attitude::A

    function PoseOptimizationParams3DOF(
            runway_corners::RC,
            observed_corners::OC,
            config::CameraConfig{S},
            noise_model,
            known_attitude::A
        ) where {T, T′, S, A<:Rotation{3}, RC<:AbstractVector{WorldPoint{T}}, OC<:AbstractVector{ProjectionPoint{T′,S}}}
        Σ = covmatrix(noise_model)
        F = cholesky(Σ)
        U = F.U
        return new{T, T′, S, A, RC, OC, typeof(U)}(runway_corners, observed_corners, config, U, known_attitude)
    end
end

"""
    pose_optimization(pose_params, p)

Unified optimization function for pose estimation.

# Arguments
- `pose_params`: Vector of pose parameters
    - `[x, y, z, roll, pitch, yaw]` for 6-DOF
    - `[x, y, z]` for 3-DOF
- `p`: `PoseOptimizationParams6DOF` or `PoseOptimizationParams3DOF`

# Returns
- Weighted reprojection error vector
"""
function pose_optimization(pose_params::AbstractVector{<:Real}, p::Union{PoseOptimizationParams6DOF, PoseOptimizationParams3DOF})
    # Unpack position parameters: [x, y, z]
    cam_pos = WorldPoint(pose_params[1:3] * u"m")

    # Determine camera rotation via pattern matching
    cam_rot = @match p begin
        q::PoseOptimizationParams6DOF => RotZYX(roll = pose_params[4] * u"rad", pitch = pose_params[5] * u"rad", yaw = pose_params[6] * u"rad")
        q::PoseOptimizationParams3DOF => q.known_attitude
    end

    # Project runway corners to image coordinates
    projected_corners = [project(cam_pos, cam_rot, corner, p.config) for corner in p.runway_corners]

    # Compute reprojection errors
    error_vectors = map(zip(projected_corners, p.observed_corners)) do (proj, obs)
        proj - obs
    end
    errors = reduce(vcat, SVector.(error_vectors))

    # Apply noise weighting via Cholesky decomposition
    U = p.chol_upper
    return ustrip.(NoUnits, (U' \ errors) / pixel)
end

"""
    _run_solver(problem_func, initial_guess, opt_params, optimization_config)

Internal helper to configure and run the nonlinear least squares solver.
"""
function _run_solver(
        problem_func::F,
        initial_guess::AbstractVector{<:Real},
        opt_params,
        optimization_config::OptimizationConfig
    ) where {F <: Function}

    prob = NonlinearLeastSquaresProblem{false}(problem_func, initial_guess, opt_params)
    termination_condition = AbsNormSafeBestTerminationMode(
        Base.Fix2(norm, 2); max_stalled_steps = 32
    )

    return solve(
        prob,
        LevenbergMarquardt();
        abstol = ustrip(optimization_config.convergence_tolerance),
        reltol = optimization_config.step_tolerance,
        maxiters = optimization_config.max_iterations,
        termination_condition,
    )
end

"""
    _solve_pose_optimization(problem_func, initial_guess, opt_params, optimization_config)

Internal helper function to solve the nonlinear least squares problem for pose estimation.
Handles both 6-DOF and 3-DOF based on `opt_params` type.
"""
function _solve_pose_optimization(
        problem_func::F,
        initial_guess::AbstractVector{<:Real},
        opt_params::Union{PoseOptimizationParams6DOF, PoseOptimizationParams3DOF},
        optimization_config::OptimizationConfig
    ) where {F <: Function}

    sol = _run_solver(problem_func, initial_guess, opt_params, optimization_config)

    # Extract common results
    position = WorldPoint(sol.u[1] * u"m", sol.u[2] * u"m", sol.u[3] * u"m")
    converged = successful_retcode(sol)
    residual_norm = norm(sol.resid) * 1pixel

    # Determine attitude and print on non-convergence for 6-DOF
    attitude = @match opt_params begin
        p::PoseOptimizationParams6DOF => begin
            a = RotZYX(roll = sol.u[4], pitch = sol.u[5], yaw = sol.u[6])
            if !converged
                println(sol.retcode)
            end
            a
        end
        p::PoseOptimizationParams3DOF => p.known_attitude
    end

    # Compute uncertainty
    uncertainty = @match opt_params begin
        p::PoseOptimizationParams6DOF => MvNormal(zeros(6), I(6))
        p::PoseOptimizationParams3DOF => MvNormal(zeros(3), I(3))
    end

    return PoseEstimate(position, attitude, uncertainty, residual_norm, converged)
end

"""
    estimate_pose_6dof(
        runway_corners::AbstractVector{<:WorldPoint},
        observed_corners::AbstractVector{<:ProjectionPoint{T, S}},
        config::CameraConfig{S};
        noise_model = UncorrGaussianNoiseModel(reduce(vcat, [SA[Normal(0.0, 2.0), Normal(0.0, 2.0)] for _ in observed_corners])),
        initial_guess_pos::AbstractVector{<:Length} = SA[-1000.0, 0.0, 100.0] * u"m",
        initial_guess_rot::AbstractVector{<:DimensionlessQuantity} = SA[0.0, 0.0, 0.0] * u"rad",
        optimization_config = DEFAULT_OPTIMIZATION_CONFIG
    ) where {T, S}

    # Default initial guesses: reasonable aircraft approach position and attitude
    initial_guess_pos = ustrip.(u"m", initial_guess_pos)
    initial_guess_rot = ustrip.(u"rad", initial_guess_rot)

    initial_guess = [initial_guess_pos; initial_guess_rot]
    opt_params = PoseOptimizationParams6DOF(runway_corners, observed_corners, config, noise_model)

    return _solve_pose_optimization(
        pose_optimization,
        initial_guess,
        opt_params,
        optimization_config
    )
end

"""
    estimate_pose_3dof(
        runway_corners::AbstractVector{<:WorldPoint},
        observed_corners::AbstractVector{<:ProjectionPoint{T, S}},
        known_attitude::RotZYX,
        config::CameraConfig{S};
        noise_model = UncorrGaussianNoiseModel(reduce(vcat, [SA[Normal(0.0, 2.0), Normal(0.0, 2.0)] for _ in observed_corners])),
        initial_guess_pos::AbstractVector{<:Length} = SA[-1000.0, 0.0, 100.0] * u"m",
        optimization_config = DEFAULT_OPTIMIZATION_CONFIG
    ) where {T, S}

    initial_guess_pos_ustrip = ustrip.(u"m", initial_guess_pos)

    opt_params = PoseOptimizationParams3DOF(
        runway_corners, observed_corners,
        config, noise_model,
        known_attitude
    )

    return _solve_pose_optimization(
        pose_optimization,
        initial_guess_pos_ustrip,
        opt_params,
        optimization_config
    )
end
