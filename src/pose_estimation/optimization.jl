"""
Pose estimation optimization using nonlinear least squares.

This module implements pose estimation by minimizing reprojection errors
using SimpleNonlinearSolve.jl and integrating with ProbabilisticParameterEstimators
noise models.
"""

using SimpleNonlinearSolve
using LinearAlgebra
using Rotations

"""
    build_pose_optimization_function(
        runway_corners::AbstractVector{<:WorldPoint},
        observed_corners::AbstractVector{<:ProjectionPoint{T, S}},
        config::CameraConfig{S},
        noise_model
    ) -> Function

Build an optimization function for pose estimation that can be passed to SimpleNonlinearSolve.

# Arguments
- `runway_corners`: 3D runway corner positions in world coordinates
- `observed_corners`: 2D observed corner positions in image coordinates  
- `config`: Camera configuration with coordinate system type
- `noise_model`: Noise model from ProbabilisticParameterEstimators

# Returns
- Function `f(pose_params, p)` suitable for SimpleNonlinearSolve

# Examples
```julia
runway_corners = [WorldPoint(0.0u"m", 25.0u"m", 0.0u"m"), ...]
observed_corners = [ProjectionPoint(100.0*1pixel, 200.0*1pixel), ...]
noise_model = UncorrGaussianNoiseModel([Normal(0.0, 2.0) for _ in 1:8])

opt_func = build_pose_optimization_function(runway_corners, observed_corners, 
                                          CAMERA_CONFIG_OFFSET, noise_model)
```
"""
function build_pose_optimization_function(
    runway_corners::AbstractVector{<:WorldPoint},
    observed_corners::AbstractVector{<:ProjectionPoint{T, S}},
    config::CameraConfig{S},
    noise_model
) where {T, S}
    
    # Build covariance matrix from noise model
    Σ = covmatrix(noise_model)
    L, U = cholesky(Σ)
    
    function optimization_function(pose_params, p)
        # Unpack pose parameters: [x, y, z, roll, pitch, yaw]
        cam_pos = WorldPoint(pose_params[1], pose_params[2], pose_params[3])
        cam_rot = RotZYX(roll=pose_params[4], pitch=pose_params[5], yaw=pose_params[6])
        
        # Project runway corners to image coordinates
        projected_corners = [project(cam_pos, cam_rot, corner, config) for corner in runway_corners]
        
        # Compute reprojection errors
        errors = collect(Iterators.flatten(
            map(zip(projected_corners, observed_corners)) do (proj, obs)
                [ustrip(proj.x - obs.x), ustrip(proj.y - obs.y)]
            end
        ))
        
        # Apply noise weighting via Cholesky decomposition
        return U' \ errors
    end
    
    return optimization_function
end


"""
    estimate_pose_6dof(
        runway_corners::AbstractVector{<:WorldPoint},
        observed_corners::AbstractVector{<:ProjectionPoint{T, S}},
        config::CameraConfig{S};
        noise_model = nothing,
        initial_guess = nothing,
        optimization_config = DEFAULT_OPTIMIZATION_CONFIG
    ) -> PoseEstimate

Estimate 6-DOF aircraft pose (position + orientation) from runway corner observations.

# Arguments
- `runway_corners`: 3D runway corner positions in world coordinates
- `observed_corners`: 2D observed corner positions in image coordinates
- `config`: Camera configuration with coordinate system type
- `noise_model`: Noise model for observations (default: 2-pixel std dev)
- `initial_guess`: Initial pose guess [x,y,z,roll,pitch,yaw] (default: reasonable guess)
- `optimization_config`: Optimization parameters

# Returns
- `PoseEstimate` with estimated pose, uncertainty, and convergence info

# Examples
```julia
runway_corners = get_runway_corners(runway_spec)
observed_corners = [ProjectionPoint(100.0*1pixel, 200.0*1pixel), ...]

pose_est = estimate_pose_6dof(runway_corners, observed_corners, CAMERA_CONFIG_OFFSET)
println("Position: ", pose_est.position)
println("Converged: ", pose_est.converged)
```
"""
function estimate_pose_6dof(
    runway_corners::AbstractVector{<:WorldPoint},
    observed_corners::AbstractVector{<:ProjectionPoint{T, S}},
    config::CameraConfig{S};
    noise_model = nothing,
    initial_guess = nothing,
    optimization_config = DEFAULT_OPTIMIZATION_CONFIG
) where {T, S}
    
    # Default noise model: 2-pixel standard deviation for each corner (x,y)
    if noise_model === nothing
        noise_dists = [Normal(0.0, 2.0) for _ in 1:(2*length(observed_corners))]
        noise_model = UncorrGaussianNoiseModel(noise_dists)
    end
    
    # Default initial guess: reasonable aircraft approach position
    if initial_guess === nothing
        initial_guess = [-1000.0, 0.0, 100.0, 0.0, 0.0, 0.0]  # [x,y,z,roll,pitch,yaw]
    end
    
    # Build optimization function
    opt_func = build_pose_optimization_function(runway_corners, observed_corners, config, noise_model)
    
    # Create and solve nonlinear least squares problem
    prob = NonlinearLeastSquaresProblem{false}(opt_func, initial_guess, nothing)
    
    sol = solve(prob, SimpleNewtonRaphson(); 
                abstol = ustrip(optimization_config.convergence_tolerance),
                reltol = optimization_config.step_tolerance,
                maxiters = optimization_config.max_iterations)
    
    # Extract results
    position = WorldPoint(sol.u[1]*u"m", sol.u[2]*u"m", sol.u[3]*u"m")
    orientation = RotZYX(roll=sol.u[4], pitch=sol.u[5], yaw=sol.u[6])
    residual_norm = norm(sol.resid) * 1pixel
    converged = sol.retcode == ReturnCode.Success
    
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
        initial_guess = nothing,
        optimization_config = DEFAULT_OPTIMIZATION_CONFIG
    ) -> PoseEstimate

Estimate 3-DOF aircraft position with known orientation from runway corner observations.

# Arguments
- `runway_corners`: 3D runway corner positions in world coordinates
- `observed_corners`: 2D observed corner positions in image coordinates
- `known_orientation`: Known aircraft orientation
- `config`: Camera configuration with coordinate system type
- `noise_model`: Noise model for observations (default: 2-pixel std dev)
- `initial_guess`: Initial position guess [x,y,z] (default: reasonable guess)
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
    initial_guess = nothing,
    optimization_config = DEFAULT_OPTIMIZATION_CONFIG
) where {T, S}
    
    # Default noise model: 2-pixel standard deviation for each corner (x,y)
    if noise_model === nothing
        noise_dists = [Normal(0.0, 2.0) for _ in 1:(2*length(observed_corners))]
        noise_model = UncorrGaussianNoiseModel(noise_dists)
    end
    
    # Default initial guess: reasonable aircraft approach position
    if initial_guess === nothing
        initial_guess = [-1000.0, 0.0, 100.0]  # [x,y,z]
    end
    
    # Build covariance matrix from noise model
    Σ = covmatrix(noise_model)
    L, U = cholesky(Σ)
    
    function optimization_function(pos_params, p)
        # Unpack position parameters: [x, y, z]
        cam_pos = WorldPoint(pos_params[1], pos_params[2], pos_params[3])
        
        # Use known orientation
        cam_rot = known_orientation
        
        # Project runway corners to image coordinates
        projected_corners = [project(cam_pos, cam_rot, corner, config) for corner in runway_corners]
        
        # Compute reprojection errors
        errors = collect(Iterators.flatten(
            map(zip(projected_corners, observed_corners)) do (proj, obs)
                [ustrip(proj.x - obs.x), ustrip(proj.y - obs.y)]
            end
        ))
        
        # Apply noise weighting via Cholesky decomposition
        return U' \ errors
    end
    
    # Create and solve nonlinear least squares problem
    prob = NonlinearLeastSquaresProblem{false}(optimization_function, initial_guess, nothing)
    
    sol = solve(prob, SimpleNewtonRaphson(); 
                abstol = ustrip(optimization_config.convergence_tolerance),
                reltol = optimization_config.step_tolerance,
                maxiters = optimization_config.max_iterations)
    
    # Extract results
    position = WorldPoint(sol.u[1]*u"m", sol.u[2]*u"m", sol.u[3]*u"m")
    residual_norm = norm(sol.resid) * 1pixel
    converged = sol.retcode == ReturnCode.Success
    
    # TODO: Compute uncertainty from Jacobian at solution
    # For now, use 3-DOF identity covariance as placeholder
    uncertainty = MvNormal(zeros(3), I(3))
    
    return PoseEstimate(position, known_orientation, uncertainty, residual_norm, converged)
end
