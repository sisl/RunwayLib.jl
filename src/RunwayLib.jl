module RunwayLib

using LinearAlgebra
using StaticArrays
using Rotations
using Unitful
using Distributions
using TypedTables
using SimpleNonlinearSolve

# Define custom pixel unit
@unit pixel "pixel" Pixel 1 false

# Register the pixel unit with Unitful
Unitful.register(RunwayLib)

# Export coordinate system types
export WorldPoint, CameraPoint, ProjectionPoint

# Export transformation functions
export world_pt_to_cam_pt, cam_pt_to_world_pt, project

# Export data structures
export RunwaySpec, PoseEstimate

# Re-export noise models from ProbabilisticParameterEstimators
using ProbabilisticParameterEstimators: UncorrGaussianNoiseModel, CorrGaussianNoiseModel, covmatrix
export UncorrGaussianNoiseModel, CorrGaussianNoiseModel, covmatrix

# Export camera model functions
export get_focal_length_pixels, get_field_of_view, pixel_to_ray_direction

# Export runway database functions
export get_runway_corners, validate_runway_spec

# Export TypedTables for data management
export Table

# Export configuration
export CAMERA_CONFIG, CAMERA_CONFIG_CENTERED, CAMERA_CONFIG_OFFSET, CameraConfig, DEFAULT_OPTIMIZATION_CONFIG

# Export custom units
export pixel

# Include submodules
include("coordinate_systems/types.jl")
include("coordinate_systems/transformations.jl")
include("camera_model/projection.jl")
include("data_management/runway_database.jl")
include("pose_estimation/types.jl")
include("pose_estimation/optimization.jl")

# Export pose estimation functions
export estimate_pose_6dof, estimate_pose_3dof
export build_pose_optimization_function

function compute_raim_statistic(pose_estimate, runway_spec, corners, noise_model)
    error("compute_raim_statistic not yet implemented")
end

function check_integrity(raim_statistic; significance_level=0.05)
    error("check_integrity not yet implemented")
end

function load_runway_database(filename)
    error("load_runway_database not yet implemented")
end

function load_flight_data(filename)
    error("load_flight_data not yet implemented - will return TypedTables.Table")
end

function extract_runway_corners(flight_data_row)
    error("extract_runway_corners not yet implemented - accepts TypedTables row")
end

function extract_uncertainties(flight_data_row)
    error("extract_uncertainties not yet implemented - accepts TypedTables row")
end

end # module RunwayLib
