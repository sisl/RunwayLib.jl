module RunwayPoseEstimation

using LinearAlgebra
using Distributions
using Rotations
using StaticArrays
using NonlinearSolve
using ForwardDiff

# Re-export from ProbabilisticParameterEstimators
using ..lib.ProbabilisticParameterEstimators
export UncorrGaussianNoiseModel, CorrGaussianNoiseModel, UncorrProductNoiseModel
export MCMCEstimator, LSQEstimator, LinearApproxEstimator
export predictsamples, predictdist
export mvnoisedistribution, covmatrix

# Core coordinate system types
export WorldPoint, CameraPoint, ProjectionPoint

# Data structures
export RunwaySpec, PoseEstimate

# Camera configuration
export CAMERA_CONFIG, ESTIMATION_CONFIG

# Core functions (to be implemented)
export world_pt_to_cam_pt, cam_pt_to_world_pt, project
export estimate_pose_6dof, estimate_pose_3dof
export compute_raim_statistic, check_integrity
export load_runway_database, load_flight_data
export extract_runway_corners, extract_uncertainties

# Coordinate system types
struct WorldPoint{T <: Real} <: FieldVector{3, T}
    x::T  # Along-track distance
    y::T  # Cross-track distance  
    z::T  # Height above runway
end

struct CameraPoint{T <: Real} <: FieldVector{3, T}
    x::T  # Camera forward direction
    y::T  # Camera right direction
    z::T  # Camera down direction
end

struct ProjectionPoint{T <: Real} <: FieldVector{2, T}
    x::T  # Image x-coordinate (pixels)
    y::T  # Image y-coordinate (pixels)
end

# Data structures
struct RunwaySpec
    icao_code::String
    length_m::Float64
    width_m::Float64
    threshold_elevation_m::Float64
    true_bearing_deg::Float64
end

struct PoseEstimate
    position::WorldPoint{Float64}
    orientation::RotZYX{Float64}
    uncertainty::MvNormal  # Joint position-orientation uncertainty
    residual_norm::Float64
    converged::Bool
end

# Configuration constants
const CAMERA_CONFIG = (
    focal_length_m = 25e-3,           # 25mm focal length
    pixel_size_m = 0.00345e-3,        # Pixel physical size
    image_width_px = 4096,            # Image width in pixels
    image_height_px = 3000,           # Image height in pixels
    optical_center_u_px = 2047.5,     # Principal point x
    optical_center_v_px = 1499.5      # Principal point y
)

const ESTIMATION_CONFIG = (
    max_iterations = 100,
    convergence_tolerance = 1e-6,
    initial_guess_noise_std = [100.0, 10.0, 5.0, 0.1, 0.1, 0.1], # pos + rot
    integrity_significance_level = 0.05
)

# Placeholder functions (to be implemented in separate files)
function world_pt_to_cam_pt(cam_pos::WorldPoint, cam_rot::RotZYX, pt::WorldPoint)
    R = cam_rot; t = cam_pos
    CameraPoint(R' * (pt - t))
end

function cam_pt_to_world_pt(cam_pos::WorldPoint, cam_rot::RotZYX, pt::CameraPoint)
    R = cam_rot; t = cam_pos
    R * pt + t
end

function project(cam_pos::WorldPoint, cam_rot::RotZYX, pt::WorldPoint;
                 focal_length::Real=CAMERA_CONFIG.focal_length_m)
    pt_ = world_pt_to_cam_pt(cam_pos, cam_rot, pt)
    scale = let focal_length = CAMERA_CONFIG.focal_length_m, 
                pixel_size = CAMERA_CONFIG.pixel_size_m
        focal_length / pixel_size
    end
    ProjectionPoint((scale .* pt_[2:3] ./ pt_[1])...)
end

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

function check_integrity(raim_statistic; significance_level=ESTIMATION_CONFIG.integrity_significance_level)
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

# Include submodules (to be created)
# include("data_management/runway_database.jl")
# include("data_management/flight_data.jl")
# include("coordinate_systems/transformations.jl")
# include("camera_model/projection.jl")
# include("pose_estimation/six_dof.jl")
# include("pose_estimation/three_dof.jl")
# include("uncertainty_quantification/noise_models.jl")
# include("integrity_monitoring/raim.jl")
# include("visualization/plotting.jl")

end # module RunwayPoseEstimation
