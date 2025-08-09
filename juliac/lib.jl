using RunwayLib
using StaticArrays
using Rotations
using Unitful, Unitful.DefaultSymbols

# Import the types and functions we need
using RunwayLib: WorldPoint, ProjectionPoint, CAMERA_CONFIG_OFFSET, CAMERA_CONFIG_CENTERED
using RunwayLib: project, estimatepose6dof, estimatepose3dof

# C struct definitions for interop
struct WorldPoint_C
    x::Float64
    y::Float64
    z::Float64
end

struct ProjectionPoint_C
    x::Float64
    y::Float64
end

struct Rotation_C
    yaw::Float64
    pitch::Float64
    roll::Float64
end

struct PoseEstimate_C
    position::WorldPoint_C
    rotation::Rotation_C
    residual_norm::Float64
    converged::Cint
end

# Error codes
const POSEEST_SUCCESS = 0
const POSEEST_ERROR_INVALID_INPUT = -1
const POSEEST_ERROR_BEHIND_CAMERA = -2
const POSEEST_ERROR_NO_CONVERGENCE = -3
const POSEEST_ERROR_INSUFFICIENT_POINTS = -4

# Global variable to track library initialization
const LIBRARY_INITIALIZED = Ref(false)

# Conversion functions
function worldpoint_c_to_jl(wpt_c::WorldPoint_C)
    return WorldPoint(wpt_c.x * m, wpt_c.y * m, wpt_c.z * m)
end

function worldpoint_jl_to_c(wpt::WorldPoint)
    return WorldPoint_C(ustrip(m, wpt.x), ustrip(m, wpt.y), ustrip(m, wpt.z))
end

function projectionpoint_c_to_jl(ppt_c::ProjectionPoint_C)
    return ProjectionPoint{Float64, :offset}(ppt_c.x, ppt_c.y)
end

function projectionpoint_jl_to_c(ppt::ProjectionPoint)
    return ProjectionPoint_C(ustrip(pixel, ppt.x), ustrip(pixel, ppt.y))
end

function rotation_c_to_jl(rot_c::Rotation_C)
    return RotZYX(rot_c.yaw, rot_c.pitch, rot_c.roll)
end

function rotation_jl_to_c(rot::RotZYX)
    return Rotation_C(rot.theta1, rot.theta2, rot.theta3)  # yaw, pitch, roll
end

function get_camera_config(config_type::Cint)
    if config_type == 0  # CAMERA_CONFIG_CENTERED
        return CAMERA_CONFIG_CENTERED
    elseif config_type == 1  # CAMERA_CONFIG_OFFSET
        return CAMERA_CONFIG_OFFSET
    else
        throw(ArgumentError("Invalid camera config type: $config_type"))
    end
end

# Error message function
function get_error_message_impl(error_code::Int)
    messages = Dict(
        POSEEST_SUCCESS => "Success",
        POSEEST_ERROR_INVALID_INPUT => "Invalid input parameters",
        POSEEST_ERROR_BEHIND_CAMERA => "Point is behind camera",
        POSEEST_ERROR_NO_CONVERGENCE => "Optimization did not converge",
        POSEEST_ERROR_INSUFFICIENT_POINTS => "Insufficient number of points"
    )
    return get(messages, error_code, "Unknown error")
end

# Store error messages in a global to ensure they persist
const ERROR_MESSAGES = Dict{Int, Ptr{UInt8}}()

Base.@ccallable function get_error_message(error_code::Cint)::Ptr{UInt8}
    if !haskey(ERROR_MESSAGES, error_code)
        msg = get_error_message_impl(error_code)
        ERROR_MESSAGES[error_code] = pointer(msg)
    end
    return ERROR_MESSAGES[error_code]
end

# Library initialization
Base.@ccallable function initialize_poseest_library(depot_path::Ptr{UInt8})::Cint
    try
        if depot_path != C_NULL
            path_str = unsafe_string(depot_path)
            ENV["JULIA_DEPOT_PATH"] = path_str
        end
        LIBRARY_INITIALIZED[] = true
        return POSEEST_SUCCESS
    catch e
        println(stderr, "Failed to initialize library: $e")
        return POSEEST_ERROR_INVALID_INPUT
    end
end

# Enhanced 6DOF pose estimation
Base.@ccallable function estimate_pose_6dof(
    runway_corners::Ptr{WorldPoint_C},
    projections::Ptr{ProjectionPoint_C},
    num_points::Cint,
    camera_config::Cint,
    result::Ptr{PoseEstimate_C}
)::Cint
    try
        # Validate inputs
        if runway_corners == C_NULL || projections == C_NULL || result == C_NULL
            return POSEEST_ERROR_INVALID_INPUT
        end
        
        if num_points < 4
            return POSEEST_ERROR_INSUFFICIENT_POINTS
        end

        # Convert C arrays to Julia arrays
        corners_c = unsafe_wrap(Array, runway_corners, num_points)
        projs_c = unsafe_wrap(Array, projections, num_points)
        
        # Convert to Julia types
        jl_corners = [worldpoint_c_to_jl(corner) for corner in corners_c]
        jl_projections = [projectionpoint_c_to_jl(proj) for proj in projs_c]
        
        # Get camera configuration
        camconfig = get_camera_config(camera_config)
        
        # Perform pose estimation
        sol = estimatepose6dof(jl_corners, jl_projections, camconfig)
        
        # Convert result back to C struct
        result_c = PoseEstimate_C(
            worldpoint_jl_to_c(sol.pos),
            rotation_jl_to_c(sol.rot),
            # Note: sol doesn't seem to have residual_norm and converged fields in the current implementation
            # We'll set defaults for now
            0.0,  # residual_norm
            1     # converged (assume success if no exception)
        )
        
        # Write result to output pointer
        unsafe_store!(result, result_c)
        
        return POSEEST_SUCCESS
        
    catch e
        println(stderr, "Error in estimate_pose_6dof: $e")
        if isa(e, BoundsError) || isa(e, ArgumentError)
            return POSEEST_ERROR_INVALID_INPUT
        else
            return POSEEST_ERROR_NO_CONVERGENCE
        end
    end
end

# Enhanced 3DOF pose estimation
Base.@ccallable function estimate_pose_3dof(
    runway_corners::Ptr{WorldPoint_C},
    projections::Ptr{ProjectionPoint_C},
    num_points::Cint,
    known_rotation::Ptr{Rotation_C},
    camera_config::Cint,
    result::Ptr{PoseEstimate_C}
)::Cint
    try
        # Validate inputs
        if runway_corners == C_NULL || projections == C_NULL || known_rotation == C_NULL || result == C_NULL
            return POSEEST_ERROR_INVALID_INPUT
        end
        
        if num_points < 3
            return POSEEST_ERROR_INSUFFICIENT_POINTS
        end

        # Convert C arrays to Julia arrays
        corners_c = unsafe_wrap(Array, runway_corners, num_points)
        projs_c = unsafe_wrap(Array, projections, num_points)
        known_rot_c = unsafe_load(known_rotation)
        
        # Convert to Julia types
        jl_corners = [worldpoint_c_to_jl(corner) for corner in corners_c]
        jl_projections = [projectionpoint_c_to_jl(proj) for proj in projs_c]
        jl_rotation = rotation_c_to_jl(known_rot_c)
        
        # Get camera configuration
        camconfig = get_camera_config(camera_config)
        
        # Perform pose estimation
        sol = estimatepose3dof(jl_corners, jl_projections, jl_rotation, camconfig)
        
        # Convert result back to C struct
        result_c = PoseEstimate_C(
            worldpoint_jl_to_c(sol.pos),
            rotation_jl_to_c(jl_rotation),  # Use known rotation
            0.0,  # residual_norm
            1     # converged
        )
        
        # Write result to output pointer
        unsafe_store!(result, result_c)
        
        return POSEEST_SUCCESS
        
    catch e
        println(stderr, "Error in estimate_pose_3dof: $e")
        if isa(e, BoundsError) || isa(e, ArgumentError)
            return POSEEST_ERROR_INVALID_INPUT
        else
            return POSEEST_ERROR_NO_CONVERGENCE
        end
    end
end

# Point projection utility
Base.@ccallable function project_point(
    camera_position::Ptr{WorldPoint_C},
    camera_rotation::Ptr{Rotation_C},
    world_point::Ptr{WorldPoint_C},
    camera_config::Cint,
    result::Ptr{ProjectionPoint_C}
)::Cint
    try
        # Validate inputs
        if camera_position == C_NULL || camera_rotation == C_NULL || world_point == C_NULL || result == C_NULL
            return POSEEST_ERROR_INVALID_INPUT
        end
        
        # Load C structs
        cam_pos_c = unsafe_load(camera_position)
        cam_rot_c = unsafe_load(camera_rotation)
        world_pt_c = unsafe_load(world_point)
        
        # Convert to Julia types
        jl_cam_pos = worldpoint_c_to_jl(cam_pos_c)
        jl_cam_rot = rotation_c_to_jl(cam_rot_c)
        jl_world_pt = worldpoint_c_to_jl(world_pt_c)
        
        # Get camera configuration
        camconfig = get_camera_config(camera_config)
        
        # Project point
        jl_projection = project(jl_cam_pos, jl_cam_rot, jl_world_pt, camconfig)
        
        # Convert result back to C struct
        result_c = projectionpoint_jl_to_c(jl_projection)
        
        # Write result to output pointer
        unsafe_store!(result, result_c)
        
        return POSEEST_SUCCESS
        
    catch e
        println(stderr, "Error in project_point: $e")
        if isa(e, RunwayLib.BehindCameraException)
            return POSEEST_ERROR_BEHIND_CAMERA
        elseif isa(e, BoundsError) || isa(e, ArgumentError)
            return POSEEST_ERROR_INVALID_INPUT
        else
            return POSEEST_ERROR_NO_CONVERGENCE
        end
    end
end

# Original test function (keeping for compatibility)
Base.@ccallable function test_estimators()::Cint
    runway_corners = SA[
        WorldPoint(0.0m, -50.0m, 0.0m),
        WorldPoint(0.0m, 50.0m, 0.0m),
        WorldPoint(3000.0m, 50.0m, 0.0m),
        WorldPoint(3000.0m, -50.0m, 0.0m),
    ]
    true_pos = WorldPoint(-1300.0m, 0.0m, 80.0m)
    true_rot = RotZYX(roll = 0.03, pitch = 0.04, yaw = 0.05)
    camconfig = CAMERA_CONFIG_OFFSET
    projections = [project(true_pos, true_rot, corner, camconfig) + ProjectionPoint(randn(2))*px
                   for corner in runway_corners]

    sol6dof = estimatepose6dof(
        runway_corners, projections, camconfig
    )

    # sol3dof = estimatepose3dof(
    #     runway_corners, projections, true_rot, camconfig
    # )
    println(Core.stdout, ustrip(m, sol6dof.pos.x))
    println(Core.stdout, ustrip(m, sol6dof.pos.y))
    println(Core.stdout, ustrip(m, sol6dof.pos.z))
    return 0
end
