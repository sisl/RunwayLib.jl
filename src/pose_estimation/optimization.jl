"""
Pose estimation optimization using nonlinear least squares.

This module implements pose estimation by minimizing reprojection errors
using SimpleNonlinearSolve.jl and integrating with ProbabilisticParameterEstimators
noise models.
"""

using SimpleNonlinearSolve
import NonlinearSolveFirstOrder: LevenbergMarquardtTrustRegion, LevenbergMarquardt
import SciMLBase: successful_retcode
using LinearAlgebra
using Rotations

"""
    PoseOptimizationParams{T, S}

Parameters for pose optimization that get passed through the parameter interface.

# Fields
- `runway_corners`: 3D runway corner positions in world coordinates
- `observed_corners`: 2D observed corner positions in image coordinates
- `config`: Camera configuration with coordinate system type
- `chol_upper`: Upper triangular matrix from Cholesky decomposition of noise covariance
- `known_attitude`: Optional known attitude for 3-DOF estimation
"""
struct PoseOptimizationParams{T, T′, S, A <: Union{Nothing, <:Rotation{3}}}
    runway_corners::AbstractVector{<:WorldPoint{T}}
    observed_corners::AbstractVector{<:ProjectionPoint{T′, S}}
    config::CameraConfig{S}
    chol_upper::AbstractMatrix{Float64}
    known_attitude::A
end

function PoseOptimizationParams(
        runway_corners::AbstractVector{<:WorldPoint{T}},
        observed_corners::AbstractVector{<:ProjectionPoint{T′, S}},
        config::CameraConfig{S},
        noise_model;
        known_attitude = nothing
    ) where {T, T′, S}
    Σ = covmatrix(noise_model)
    L, U = cholesky(Σ)
    return PoseOptimizationParams{T, T′, S}(runway_corners, observed_corners, config, U, known_attitude)
end

"""
    pose_optimization_6dof(pose_params, p::PoseOptimizationParams)

Optimization function for 6-DOF pose estimation (position + attitude).

# Arguments
- `pose_params`: Vector [x, y, z, roll, pitch, yaw] of pose parameters
- `p`: PoseOptimizationParams containing problem data

# Returns
- Weighted reprojection error vector
"""
function pose_optimization_6dof(pose_params, p::PoseOptimizationParams)
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
    pose_optimization_3dof(pos_params, p::PoseOptimizationParams)

Optimization function for 3-DOF position estimation with known attitude.

# Arguments
- `pos_params`: Vector [x, y, z] of position parameters
- `p`: PoseOptimizationParams containing problem data (must have known_attitude set)

# Returns
- Weighted reprojection error vector
"""
function pose_optimization_3dof(pos_params, p::PoseOptimizationParams)
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

    # Combine into single initial guess vector
    initial_guess = [initial_guess_pos; initial_guess_rot]

    # Create optimization parameters
    opt_params = PoseOptimizationParams(runway_corners, observed_corners, config, noise_model)

    # Create and solve nonlinear least squares problem
    prob = NonlinearLeastSquaresProblem{false}(pose_optimization_6dof, initial_guess, opt_params)
    termination_condition = AbsNormSafeBestTerminationMode(
        Base.Fix2(norm, 2); max_stalled_steps = 32
    )

    sol = solve(
        prob,
        # SimpleTrustRegion();
        # LevenbergMarquardtTrustRegion(1.0);
        LevenbergMarquardt();
        abstol = ustrip(optimization_config.convergence_tolerance),
        reltol = optimization_config.step_tolerance,
        maxiters = optimization_config.max_iterations,
        termination_condition,
    )

    # Extract results
    position = WorldPoint(sol.u[1] * u"m", sol.u[2] * u"m", sol.u[3] * u"m")
    attitude = RotZYX(roll = sol.u[4], pitch = sol.u[5], yaw = sol.u[6])
    residual_norm = norm(sol.resid) * 1pixel
    converged = successful_retcode(sol)
    if !converged
        println(sol.retcode)
    end

    # TODO: Compute uncertainty from Jacobian at solution
    # For now, use identity covariance as placeholder
    uncertainty = MvNormal(zeros(6), I(6))

    return PoseEstimate(position, attitude, uncertainty, residual_norm, converged)
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

    initial_guess_pos = ustrip.(u"m", initial_guess_pos)

    # Create optimization parameters with known attitude
    opt_params = PoseOptimizationParams(
        runway_corners, observed_corners,
        config, noise_model;
        known_attitude = known_attitude
    )

    # Create and solve nonlinear least squares problem
    prob = NonlinearLeastSquaresProblem{false}(pose_optimization_3dof, initial_guess_pos, opt_params)

    termination_condition = AbsNormSafeBestTerminationMode(
        Base.Fix2(norm, 2); max_stalled_steps = 32
    )

    sol = solve(
        prob,
        # SimpleTrustRegion();
        # LevenbergMarquardtTrustRegion(1.0);
        LevenbergMarquardt();
        abstol = ustrip(optimization_config.convergence_tolerance),
        reltol = optimization_config.step_tolerance,
        maxiters = optimization_config.max_iterations,
        termination_condition,
    )

    # Extract results
    position = WorldPoint(sol.u[1] * u"m", sol.u[2] * u"m", sol.u[3] * u"m")
    residual_norm = norm(sol.resid) * 1pixel
    converged = successful_retcode(sol)

    # TODO: Compute uncertainty from Jacobian at solution
    # For now, use 3-DOF identity covariance as placeholder
    uncertainty = MvNormal(zeros(3), I(3))

    return PoseEstimate(position, known_attitude, uncertainty, residual_norm, converged)
end
