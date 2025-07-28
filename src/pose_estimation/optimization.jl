"""
Pose estimation optimization using nonlinear least squares.

This module implements pose estimation by minimizing reprojection errors
using SimpleNonlinearSolve.jl and integrating with ProbabilisticParameterEstimators
noise models.
"""

using SimpleNonlinearSolve
import Moshi.Match: @match
import NonlinearSolveFirstOrder: LevenbergMarquardtTrustRegion, LevenbergMarquardt
import SciMLBase: successful_retcode
using StaticArrays
using LinearAlgebra
import LinearAlgebra: AbstractTriangular
using Rotations

abstract type AbstractPoseOptimizationParams end

"""
    PoseOptimizationParams6DOF{T, T′, S, RC, OC, M}

Parameters for 6-DOF pose optimization (position + attitude).
"""
struct PoseOptimizationParams6DOF{T, T′, S,
        RC <: AbstractVector{WorldPoint{T}},
        OC <: AbstractVector{ProjectionPoint{T′, S}},
        M  <: AbstractTriangular{Float64, <:AbstractMatrix{Float64}}
    } <: AbstractPoseOptimizationParams
    runway_corners::RC
    observed_corners::OC
    camconfig::CameraConfig{S}
    chol_upper::M
end

"""
    PoseOptimizationParams3DOF{T, T′, S, A, RC, OC, M}

Parameters for 3-DOF pose optimization (position only with known attitude).
"""
struct PoseOptimizationParams3DOF{T, T′, S,
        A  <: Rotation{3},
        RC <: AbstractVector{WorldPoint{T}},
        OC <: AbstractVector{ProjectionPoint{T′, S}},
        M  <: AbstractTriangular{Float64, <:AbstractMatrix{Float64}}
    } <: AbstractPoseOptimizationParams
    runway_corners::RC
    observed_corners::OC
    camconfig::CameraConfig{S}
    chol_upper::M
    known_attitude::A
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
function pose_optimization_objective(optvar::AbstractVector{<:Real},
            ps::AbstractPoseOptimizationParams)
    cam_pos = WorldPoint(optvar[1:3]m)

    # Determine camera rotation via pattern matching
    cam_rot = @match ps begin
        ps::PoseOptimizationParams6DOF =>
            RotZYX(roll = optvar[4]rad,
                   pitch = optvar[5]rad,
                   yaw = optvar[6]rad)
        ps::PoseOptimizationParams3DOF =>
            ps.known_attitude
    end

    # Project runway corners to image coordinates
    projected_corners = [project(cam_pos, cam_rot, corner, ps.camconfig)
                         for corner in ps.runway_corners]

    # Compute reprojection errors
    error_vectors = [
        (proj - obs)
        for (proj, obs) in zip(projected_corners, p.observed_corners)
    ]
    errors = reduce(vcat, SVector.(error_vectors))

    # Apply noise weighting via Cholesky decomposition
    U = ps.chol_upper
    return ustrip.(NoUnits, (U' \ errors) / pixel)
end

const poseoptfn = NonlinearFunction{false}(pose_optimization_objective,
        AutoForwardDiff(; chunksize=1))

const _default6dofnoisemodel(pts) =
    UncorrGaussianNoiseModel(
            reduce(vcat, [SA[Normal(0.0, 2.0),
                             Normal(0.0, 2.0)]
                          for _ in pts])
    )
function estimate_pose_6dof(
        runway_corners::AbstractVector{<:WorldPoint},
        observed_corners::AbstractVector{<:ProjectionPoint{T, S}},
        config::CameraConfig{S};
        noise_model = _default6dofnoisemodel(observed_corners),
        initial_guess_pos::AbstractVector{<:Length} = SA[-1000.0, 0.0, 100.0]m,
        initial_guess_rot::AbstractVector{<:DimensionlessQuantity} = SA[0.0, 0.0, 0.0]rad,
        optimization_config = DEFAULT_OPTIMIZATION_CONFIG
    ) where {T, S}

    # Default initial guesses: reasonable aircraft approach position and attitude
    initial_guess_pos = initial_guess_pos .|> ustrip(rad)
    initial_guess_rot = initial_guess_rot .|> ustrip(rad)

    initial_guess = [initial_guess_pos; initial_guess_rot]
    opt_params = PoseOptimizationParams6DOF(
        runway_corners, observed_corners, config, noise_model)

    return _solve_pose_optimization(pose_optimization,
                                    initial_guess,
                                    opt_params,
                                    optimization_config)
    u₀ = [initial_guess_pos .|> _ustrip(m);
          initial_guess_ros .|> _ustrip(rad)]


    ps = PoseOptimizationParams6DOF(
        runway_corners, observed_corners,
        config, noise_model)


    prob = NonlinearLeastSquaresProblem{false}(poseoptfn, u₀, ps)
    alg = LevenbergMarquardtTrustRegion()
    # termination_condition = AbsNormSafeBestTerminationMode(
    #     Base.Fix2(norm, 2);
    #     max_stalled_steps = 32)

    sol = solve(prob, alg)

    !recode_successful(sol) || throw(OptimizationFailedError(sol.retcode, sol))
    pos = WorldPoint(sol.u[1:3]m)
    rot = RotZYX(roll=sol.u[4]rad, pitch=sol.u[5]rad, yaw=sol.u[6]rad)
    return (; pos, rot)
end

const _default3dofnoisemodel(pts) =
    UncorrGaussianNoiseModel(
            reduce(vcat, [SA[Normal(0.0, 2.0),
                             Normal(0.0, 2.0)]
                          for _ in pts])
    )
function estimate_pose_3dof(
        runway_corners::AbstractVector{<:WorldPoint},
        observed_corners::AbstractVector{<:ProjectionPoint{T, S}},
        known_attitude::RotZYX,
        config::CameraConfig{S};
        noise_model = _default3dofnoisemodel(observed_corners),
        initial_guess_pos::AbstractVector{<:Length} = SA[-1000.0, 0.0, 100.0] * u"m",
        optimization_config = DEFAULT_OPTIMIZATION_CONFIG
    ) where {T, S}

    u₀ = initial_guess_pos .|> _ustrip(m)

    ps = PoseOptimizationParams3DOF(
        runway_corners, observed_corners,
        config, noise_model, known_attitude)


    prob = NonlinearLeastSquaresProblem{false}(poseoptfn, u₀, ps)
    alg = LevenbergMarquardtTrustRegion()
    # termination_condition = AbsNormSafeBestTerminationMode(
    #     Base.Fix2(norm, 2);
    #     max_stalled_steps = 32)

    sol = solve(prob, alg)

    !recode_successful(sol) || throw(OptimizationFailedError(sol.retcode, sol))
    pos = WorldPoint(sol.u[1:3]m)
    rot = known_attitude
    return (; pos, rot)
end
