"""
Pose estimation optimization using nonlinear least squares.

This module implements pose estimation by minimizing reprojection errors
using SimpleNonlinearSolve.jl and integrating with ProbabilisticParameterEstimators
noise models.
"""

using SimpleNonlinearSolve
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
- `known_orientation`: Optional known orientation for 3-DOF estimation
"""
struct PoseOptimizationParams{T, T′, S}
    runway_corners::AbstractVector{<:WorldPoint{T}}
    observed_corners::AbstractVector{<:ProjectionPoint{T′, S}}
    config::CameraConfig{S}
    chol_upper::AbstractMatrix{Float64}
    known_orientation::Union{Nothing, RotZYX}
end

function PoseOptimizationParams(
        runway_corners::AbstractVector{<:WorldPoint{T}},
        observed_corners::AbstractVector{<:ProjectionPoint{T′, S}},
        config::CameraConfig{S},
        noise_model;
        known_orientation = nothing
    ) where {T, T′, S}
    Σ = covmatrix(noise_model)
    L, U = cholesky(Σ)
    return PoseOptimizationParams{T, T′, S}(runway_corners, observed_corners, config, U, known_orientation)
end

"""
    pose_optimization_6dof(pose_params, p::PoseOptimizationParams)

Optimization function for 6-DOF pose estimation (position + orientation).

# Arguments
- `pose_params`: Vector [x, y, z, roll, pitch, yaw] of pose parameters
- `p`: PoseOptimizationParams containing problem data

# Returns
- Weighted reprojection error vector
"""
function pose_optimization_6dof(pose_params, p::PoseOptimizationParams)
    # Unpack pose parameters: [x, y, z, roll, pitch, yaw]
    cam_pos = WorldPoint(pose_params[1], pose_params[2], pose_params[3])
    cam_rot = RotZYX(roll = pose_params[4], pitch = pose_params[5], yaw = pose_params[6])

    # Project runway corners to image coordinates
    projected_corners = [project(cam_pos, cam_rot, corner, p.config) for corner in p.runway_corners]

    # Compute reprojection errors
    error_vectors = map(zip(projected_corners, p.observed_corners)) do (proj, obs)
        ustrip.(proj - obs)
    end
    errors = SVector{2 * length(p.runway_corners)}(Iterators.flatten(error_vectors)...)

    # Apply noise weighting via Cholesky decomposition
    return p.chol_upper' \ errors
end

"""
    pose_optimization_3dof(pos_params, p::PoseOptimizationParams)

Optimization function for 3-DOF position estimation with known orientation.

# Arguments
- `pos_params`: Vector [x, y, z] of position parameters
- `p`: PoseOptimizationParams containing problem data (must have known_orientation set)

# Returns
- Weighted reprojection error vector
"""
function pose_optimization_3dof(pos_params, p::PoseOptimizationParams)
    # Unpack position parameters: [x, y, z]
    cam_pos = WorldPoint(pos_params[1] * u"m", pos_params[2] * u"m", pos_params[3] * u"m")

    # Use known orientation
    cam_rot = p.known_orientation

    # Project runway corners to image coordinates
    projected_corners = [project(cam_pos, cam_rot, corner, p.config) for corner in p.runway_corners]

    # Compute reprojection errors
    error_vectors = map(zip(projected_corners, p.observed_corners)) do (proj, obs)
        ustrip.(proj - obs)
    end
    errors = SVector{2 * length(p.runway_corners)}(Iterators.flatten(error_vectors)...)

    # Apply noise weighting via Cholesky decomposition
    return ustrip.((p.chol_upper' * pixel) \ errors)
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

Estimate 6-DOF aircraft pose (position + orientation) from runway corner observations.

# Arguments
- `runway_corners`: 3D runway corner positions in world coordinates
- `observed_corners`: 2D observed corner positions in image coordinates
- `config`: Camera configuration with coordinate system type
- `noise_model`: Noise model for observations (default: 2-pixel std dev)
- `initial_guess_pos`: Initial position guess [x,y,z] with units (default: reasonable guess)
- `initial_guess_rot`: Initial rotation guess [roll,pitch,yaw] in radians (default: reasonable guess)
- `optimization_config`: Optimization parameters

# Returns
- `PoseEstimate` with estimated pose, uncertainty, and convergence info

# Examples
```julia
runway_corners = get_runway_corners(runway_spec)
observed_corners = [ProjectionPoint(100.0*1pixel, 200.0*1pixel), ...]

pose_est = estimate_pose_6dof(runway_corners, observed_corners, CAMERA_CONFIG_OFFSET;
                             initial_guess_pos = [-800.0u"m", 0.0u"m", 120.0u"m"],
                             initial_guess_rot = [0.0, 0.05, 0.0])
println("Position: ", pose_est.position)
println("Converged: ", pose_est.converged)
```
"""
function estimate_pose_6dof(
        runway_corners::AbstractVector{<:WorldPoint},
        observed_corners::AbstractVector{<:ProjectionPoint{T, S}},
        config::CameraConfig{S};
        noise_model = nothing,
        initial_guess_pos = nothing,
        initial_guess_rot = nothing,
        optimization_config = DEFAULT_OPTIMIZATION_CONFIG
    ) where {T, S}

    # Default noise model: 2-pixel standard deviation for each corner (x,y)
    if noise_model === nothing
        noise_dists = [Normal(0.0, 2.0) for _ in 1:(2 * length(observed_corners))]
        noise_model = UncorrGaussianNoiseModel(noise_dists)
    end

    # Default initial guesses: reasonable aircraft approach position and orientation
    if initial_guess_pos === nothing
        initial_guess_pos = [-1000.0, 0.0, 100.0]  # [x,y,z] in meters
    else
        initial_guess_pos = ustrip.(u"m", initial_guess_pos)  # Convert to meters
    end
    
    if initial_guess_rot === nothing
        initial_guess_rot = [0.0, 0.0, 0.0]  # [roll,pitch,yaw] in radians
    end

    # Combine into single initial guess vector
    initial_guess = [initial_guess_pos..., initial_guess_rot...]

    # Create optimization parameters
    opt_params = PoseOptimizationParams(runway_corners, observed_corners, config, noise_model)

    # Create and solve nonlinear least squares problem
    prob = NonlinearLeastSquaresProblem{false}(pose_optimization_6dof, initial_guess, opt_params)
    termination_condition = AbsNormSafeBestTerminationMode(
        Base.Fix2(norm, 2); max_stalled_steps = 32
    )

    sol = solve(
        prob, SimpleNewtonRaphson();
        abstol = ustrip(optimization_config.convergence_tolerance),
        reltol = optimization_config.step_tolerance,
        maxiters = optimization_config.max_iterations,
        termination_condition
    )

    # Extract results
    position = WorldPoint(sol.u[1] * u"m", sol.u[2] * u"m", sol.u[3] * u"m")
    orientation = RotZYX(roll = sol.u[4], pitch = sol.u[5], yaw = sol.u[6])
    residual_norm = norm(sol.resid) * 1pixel
    converged = successful_retcode(sol)

    # TODO: Compute uncertainty from Jacobian at solution
    # For now, use identity covariance as placeholder
    uncertainty = MvNormal(zeros(6), I(6))

    return PoseEstimate(position, orientation, uncertainty, residual_norm, converged)
end

"""
    estimate_pose_3dof(
        runway_corners::AbstractVector{<:WorldPoint},
        observed_corners::AbstractVector{<:ProjectionPoint{T, S}},
        known_orientation::RotZYX,
        config::CameraConfig{S};
        noise_model = nothing,
        initial_guess_pos = nothing,
        optimization_config = DEFAULT_OPTIMIZATION_CONFIG
    ) -> PoseEstimate

Estimate 3-DOF aircraft position with known orientation from runway corner observations.

# Arguments
- `runway_corners`: 3D runway corner positions in world coordinates
- `observed_corners`: 2D observed corner positions in image coordinates
- `known_orientation`: Known aircraft orientation
- `config`: Camera configuration with coordinate system type
- `noise_model`: Noise model for observations (default: 2-pixel std dev)
- `initial_guess_pos`: Initial position guess [x,y,z] with units (default: reasonable guess)
- `optimization_config`: Optimization parameters

# Returns
- `PoseEstimate` with estimated position, known orientation, and convergence info
"""
function estimate_pose_3dof(
        runway_corners::AbstractVector{<:WorldPoint},
        observed_corners::AbstractVector{<:ProjectionPoint{T, S}},
        known_orientation::RotZYX,
        config::CameraConfig{S};
        noise_model = nothing,
        initial_guess_pos = nothing,
        optimization_config = DEFAULT_OPTIMIZATION_CONFIG
    ) where {T, S}

    # Default noise model: 2-pixel standard deviation for each corner (x,y)
    if noise_model === nothing
        noise_dists = [Normal(0.0, 2.0) for _ in 1:(2 * length(observed_corners))]
        noise_model = UncorrGaussianNoiseModel(noise_dists)
    end

    # Default initial guess: reasonable aircraft approach position
    if initial_guess_pos === nothing
        initial_guess_pos = [-1000.0, 0.0, 100.0]  # [x,y,z] in meters
    else
        initial_guess_pos = ustrip.(u"m", initial_guess_pos)  # Convert to meters
    end

    # Create optimization parameters with known orientation
    opt_params = PoseOptimizationParams(
        runway_corners, observed_corners,
        config, noise_model;
        known_orientation = known_orientation
    )

    # Create and solve nonlinear least squares problem
    prob = NonlinearLeastSquaresProblem{false}(pose_optimization_3dof, initial_guess_pos, opt_params)

    termination_condition = AbsNormSafeBestTerminationMode(
        Base.Fix2(norm, 2); max_stalled_steps = 32
    )

    sol = solve(
        prob, SimpleNewtonRaphson();
        abstol = ustrip(optimization_config.convergence_tolerance),
        reltol = optimization_config.step_tolerance,
        maxiters = optimization_config.max_iterations,
        termination_condition
    )

    # Extract results
    position = WorldPoint(sol.u[1] * u"m", sol.u[2] * u"m", sol.u[3] * u"m")
    residual_norm = norm(sol.resid) * 1pixel
    converged = successful_retcode(sol)

    # TODO: Compute uncertainty from Jacobian at solution
    # For now, use 3-DOF identity covariance as placeholder
    uncertainty = MvNormal(zeros(3), I(3))

    return PoseEstimate(position, known_orientation, uncertainty, residual_norm, converged)
end
