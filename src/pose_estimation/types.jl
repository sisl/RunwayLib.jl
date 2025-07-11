"""
Pose estimation data structures and types.

This module defines the core data structures used in pose estimation,
including pose estimates with uncertainty quantification.
"""

using Rotations
using Distributions
using Unitful

"""
    PoseEstimate

Complete pose estimate with position, orientation, uncertainty, and convergence information.

# Fields
- `position::WorldPoint`: Estimated aircraft position in world coordinates
- `orientation::RotZYX`: Estimated aircraft orientation (yaw, pitch, roll)
- `uncertainty::MvNormal`: Joint position-orientation uncertainty distribution
- `residual_norm`: Final residual norm from optimization (with units)
- `converged::Bool`: Whether optimization converged successfully

# Examples
```julia
# Create pose estimate
position = WorldPoint(500.0u"m", 10.0u"m", 100.0u"m")
orientation = RotZYX(0.1, 0.05, 0.02)  # Small attitude angles
uncertainty = MvNormal(zeros(6), I(6))  # 6-DOF uncertainty
residual_norm = 0.5*1pixel
converged = true

pose_est = PoseEstimate(position, orientation, uncertainty, residual_norm, converged)

# Access components
println("Position: ", pose_est.position)
println("Orientation: ", pose_est.orientation)
println("Converged: ", pose_est.converged)
```
"""
struct PoseEstimate{P,O,U,R}
    position::P
    orientation::O
    uncertainty::U
    residual_norm::R
    converged::Bool

    function PoseEstimate(position::P, orientation::O, uncertainty::U,
        residual_norm::R, converged::Bool) where {P,O,U,R}
        new{P,O,U,R}(position, orientation, uncertainty, residual_norm, converged)
    end
end

"""
    OptimizationConfig

Configuration parameters for pose estimation optimization.

# Fields
- `max_iterations::Int`: Maximum number of optimization iterations
- `convergence_tolerance`: Convergence tolerance for residual norm
- `step_tolerance`: Minimum step size tolerance
- `gradient_tolerance`: Gradient norm tolerance for convergence

# Examples
```julia
config = OptimizationConfig(
    max_iterations = 100,
    convergence_tolerance = 1e-6*1pixel,
    step_tolerance = 1e-8,
    gradient_tolerance = 1e-6
)
```
"""
struct OptimizationConfig{CT,ST,GT}
    max_iterations::Int
    convergence_tolerance::CT
    step_tolerance::ST
    gradient_tolerance::GT

    function OptimizationConfig(max_iterations::Int, convergence_tolerance::CT,
        step_tolerance::ST, gradient_tolerance::GT) where {CT,ST,GT}
        if max_iterations <= 0
            throw(ArgumentError("max_iterations must be positive"))
        end

        new{CT,ST,GT}(max_iterations, convergence_tolerance, step_tolerance, gradient_tolerance)
    end
end

# Default optimization configuration
const DEFAULT_OPTIMIZATION_CONFIG = OptimizationConfig(
    100,           # max_iterations
    1e-6 * 1pixel,  # convergence_tolerance
    1e-8,          # step_tolerance
    1e-6           # gradient_tolerance
)
