module RunwayLib

using LinearAlgebra
using StaticArrays
using Rotations
using Unitful
using Distributions

# Export coordinate system types
export WorldPoint, CameraPoint, ProjectionPoint

# Export transformation functions
export world_pt_to_cam_pt, cam_pt_to_world_pt, project

# Export data structures
export RunwaySpec, PoseEstimate

# Export noise models
export UncorrGaussianNoiseModel, CorrGaussianNoiseModel

# Export camera model functions
export get_focal_length_pixels, get_field_of_view, pixel_to_ray_direction

# Export runway database functions
export get_runway_corners, validate_runway_spec

# Export configuration
export CAMERA_CONFIG, DEFAULT_OPTIMIZATION_CONFIG

# Include submodules
include("coordinate_systems/types.jl")
include("coordinate_systems/transformations.jl")
include("camera_model/projection.jl")
include("data_management/runway_database.jl")
include("pose_estimation/types.jl")
include("uncertainty_quantification/noise_models.jl")

# Placeholder functions for future implementation
function estimate_pose_6dof(runway_spec, corners, pixel_uncertainties; initial_guess=nothing)
    error("estimate_pose_6dof not yet implemented")
end

function estimate_pose_3dof(runway_spec, corners, known_orientation; initial_guess=nothing)
    error("estimate_pose_3dof not yet implemented")
end

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
    error("load_flight_data not yet implemented")
end

function extract_runway_corners(flight_data_row)
    error("extract_runway_corners not yet implemented")
end

function extract_uncertainties(flight_data_row)
    error("extract_uncertainties not yet implemented")
end

end # module RunwayLib
