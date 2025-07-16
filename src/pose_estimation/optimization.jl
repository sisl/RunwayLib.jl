"""
Pose estimation optimization using nonlinear least squares.

This module implements pose estimation by minimizing reprojection errors
using SimpleNonlinearSolve.jl and integrating with ProbabilisticParameterEstimators
noise models.
"""

using SimpleNonlinearSolve
import NonlinearSolveFirstOrder: LevenbergMarquardtTrustRegion, LevenbergMarquardt
import SciMLBase: successful_retcode
using StaticArrays
using LinearAlgebra
using Rotations

"""
    PoseOptimizationParams6DOF{T, T′, S}

Parameters for 6-DOF pose optimization (position + attitude).
"""
struct PoseOptimizationParams6DOF{T, T′, S}
    runway_corners::AbstractVector{<:WorldPoint{T}}
    observed_corners::AbstractVector{<:ProjectionPoint{T′, S}}
    config::CameraConfig{S}
    chol_upper::AbstractMatrix{Float64}
end

function PoseOptimizationParams6DOF(
        runway_corners::AbstractVector{<:WorldPoint{T}},
        observed_corners::AbstractVector{<:ProjectionPoint{T′, S}},
        config::CameraConfig{S},
        noise_model
    ) where {T, T′, S}
    Σ = covmatrix(noise_model)
    _, U = cholesky(Σ) # L is not used
    return PoseOptimizationParams6DOF{T, T′, S}(runway_corners, observed_corners, config, U)
end

"""
    PoseOptimizationParams3DOF{T, T′, S, A <: Rotation{3}}

Parameters for 3-DOF pose optimization (position only with known attitude).
"""
struct PoseOptimizationParams3DOF{T, T′, S, A <: Rotation{3}}
    runway_corners::AbstractVector{<:WorldPoint{T}}
    observed_corners::AbstractVector{<:ProjectionPoint{T′, S}}
    config::CameraConfig{S}
    chol_upper::AbstractMatrix{Float64}
    known_attitude::A # Now explicitly part of 3-DOF params
end

function PoseOptimizationParams3DOF(
        runway_corners::AbstractVector{<:WorldPoint{T}},
        observed_corners::AbstractVector{<:ProjectionPoint{T′, S}},
        config::CameraConfig{S},
        noise_model,
        known_attitude::A
    ) where {T, T′, S, A <: Rotation{3}}
    Σ = covmatrix(noise_model)
    _, U = cholesky(Σ)
    return PoseOptimizationParams3DOF{T, T′, S, A}(runway_corners, observed_corners, config, U, known_attitude)
end

"""
    pose_optimization_6dof(pose_params, p::PoseOptimizationParams6DOF)

Optimization function for 6-DOF pose estimation (position + attitude).

# Arguments
- `pose_params`: Vector [x, y, z, roll, pitch, yaw] of pose parameters
- `p`: PoseOptimizationParams6DOF containing problem data

# Returns
- Weighted reprojection error vector
"""
function pose_optimization_6dof(pose_params, p::PoseOptimizationParams6DOF)
    # Unpack pose parameters: [x, y, z, roll, pitch, yaw]
    cam_pos = WorldPoint(pose_params[1:3]m)
    cam_rot = RotZYX(roll = pose_params[4]rad, pitch = pose_params[5]rad, yaw = pose_params[6]rad)

    # Project runway corners to image coordinates
    projected_corners = [project(cam_pos, cam_rot, corner, p.config) for corner in p.runway_corners]

    # Compute reprojection errors
    error_vectors = map(zip(projected_corners, p.observed_corners)) do (proj, obs)
        proj - obs
    end
    errors = reduce(vcat, SVector.(error_vectors))

    U = p.chol_upper
    # Apply noise weighting via Cholesky decomposition
    return ustrip.(NoUnits, (U' \ errors) / pixel)
end

"""
    pose_optimization_3dof(pos_params, p::PoseOptimizationParams3DOF)

Optimization function for 3-DOF position estimation with known attitude.

# Arguments
- `pos_params`: Vector [x, y, z] of position parameters
- `p`: PoseOptimizationParams3DOF containing problem data

# Returns
- Weighted reprojection error vector
"""
function pose_optimization_3dof(pos_params, p::PoseOptimizationParams3DOF)
    # Unpack position parameters: [x, y, z]
    cam_pos = WorldPoint(pos_params[1:3]m)

    # Use known attitude
    cam_rot = p.known_attitude

    # Project runway corners to image coordinates
    projected_corners = [
        project(cam_pos, cam_rot, corner, p.config)
            for corner in p.runway_corners
    ]

    # Compute reprojection errors
    error_vectors = map(zip(projected_corners, p.observed_corners)) do (proj, obs)
        proj - obs
    end
    errors = reduce(vcat, SVector.(error_vectors))

    U = p.chol_upper
    # Apply noise weighting via Cholesky decomposition
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
    _solve_pose_optimization(...)

Internal helper function to solve the nonlinear least squares problem for 6-DOF pose estimation.
"""
function _solve_pose_optimization(
        problem_func::F,
        initial_guess::AbstractVector{<:Real},
        opt_params::PoseOptimizationParams6DOF,
        optimization_config::OptimizationConfig
    ) where {F <: Function}

    sol = _run_solver(problem_func, initial_guess, opt_params, optimization_config)

    # Extract results specific to 6-DOF
    position = WorldPoint(sol.u[1] * u"m", sol.u[2] * u"m", sol.u[3] * u"m")
    attitude = RotZYX(roll = sol.u[4], pitch = sol.u[5], yaw = sol.u[6])
    residual_norm = norm(sol.resid) * 1pixel
    converged = successful_retcode(sol)
    if !converged
        println(sol.retcode) # Print only for 6-DOF non-convergence
    end

    # TODO: Compute uncertainty from Jacobian at solution
    uncertainty = MvNormal(zeros(6), I(6)) # 6-DOF uncertainty

    return PoseEstimate(position, attitude, uncertainty, residual_norm, converged)
end

"""
    _solve_pose_optimization(...)

Internal helper function to solve the nonlinear least squares problem for 3-DOF pose estimation.
"""
function _solve_pose_optimization(
        problem_func::F,
        initial_guess::AbstractVector{<:Real},
        opt_params::PoseOptimizationParams3DOF,
        optimization_config::OptimizationConfig
    ) where {F <: Function}

    sol = _run_solver(problem_func, initial_guess, opt_params, optimization_config)

    # Extract results specific to 3-DOF
    position = WorldPoint(sol.u[1] * u"m", sol.u[2] * u"m", sol.u[3] * u"m")
    attitude = opt_params.known_attitude # Attitude comes directly from params
    residual_norm = norm(sol.resid) * 1pixel
    converged = successful_retcode(sol)

    # TODO: Compute uncertainty from Jacobian at solution
    uncertainty = MvNormal(zeros(3), I(3)) # 3-DOF uncertainty

    return PoseEstimate(position, attitude, uncertainty, residual_norm, converged)
end

"""
    estimate_pose_6dof(
        runway_corners::AbstractVector{<:WorldPoint},
        observed_corners::AbstractVector{<:ProjectionPoint{T, S}},
        config::CameraConfig{S};
        noise_model = nothing,
        initial_guess_pos = nothing,
        initial_guess_rot = nothing,
        optimization_config = DEFAULT_OPTIMIZATION_CONFIG
    ) -> PoseEstimate

Estimate 6-DOF aircraft pose (position + attitude) from runway corner observations.

# Arguments
- `runway_corners`: 3D runway corner positions in world coordinates
- `observed_corners`: 2D observed corner positions in image coordinates
- `config`: Camera configuration with coordinate system type
- `noise_model`: Noise model for observations (default: 2-pixel std dev)
- `initial_guess_pos`: Initial position guess [x,y,z] with length units (default: reasonable guess)
- `initial_guess_rot`: Initial attitude guess [roll,pitch,yaw] as dimensionless quantities (default: reasonable guess)
- `optimization_config`: Optimization parameters

# Returns
- `PoseEstimate` with estimated pose, uncertainty, and convergence info

# Examples
```julia
runway_corners = get_runway_corners(runway_spec)
observed_corners = [ProjectionPoint(100.0*1pixel, 200.0*1pixel), ...]

pose_est = estimate_pose_6dof(runway_corners, observed_corners, CAMERA_CONFIG_OFFSET;
                             initial_guess_pos = SA[-800.0u"m", 0.0u"m", 120.0u"m"],
                             initial_guess_rot = SA[0.0u"rad", 0.05u"rad", 0.0u"rad"])
println("Position: ", pose_est.position)
println("Converged: ", pose_est.converged)
```
"""
function estimate_pose_6dof(
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
        pose_optimization_6dof,
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
        noise_model = nothing,
        initial_guess_pos = nothing,
        optimization_config = DEFAULT_OPTIMIZATION_CONFIG
    ) -> PoseEstimate

Estimate 3-DOF aircraft position with known attitude from runway corner observations.

# Arguments
- `runway_corners`: 3D runway corner positions in world coordinates
- `observed_corners`: 2D observed corner positions in image coordinates
- `known_attitude`: Known aircraft attitude
- `config`: Camera configuration with coordinate system type
- `noise_model`: Noise model for observations (default: 2-pixel std dev)
- `initial_guess_pos`: Initial position guess [x,y,z] with length units (default: reasonable guess)
- `optimization_config`: Optimization parameters

# Returns
- `PoseEstimate` with estimated position, known attitude, and convergence info
"""
function estimate_pose_3dof(
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
        pose_optimization_3dof,
        initial_guess_pos_ustrip,
        opt_params,
        optimization_config
    )
end
