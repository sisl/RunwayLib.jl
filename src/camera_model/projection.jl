"""
Camera projection model for runway pose estimation.

This module implements the pinhole camera model for projecting 3D points
to 2D image coordinates, with support for units and realistic camera parameters.
"""

using LinearAlgebra
using Unitful
import Moshi.Match: @match

# Camera configuration with type parameter for coordinate system
struct CameraConfig{S}
    focal_length::typeof(1.0 * u"mm")
    pixel_size::typeof(1.0 * u"μm" / 1pixel)
    image_width::typeof(1pixel)
    image_height::typeof(1pixel)
    optical_center_u::typeof(1.0pixel)
    optical_center_v::typeof(1.0pixel)
end

# Default camera configurations
const CAMERA_CONFIG_CENTERED = CameraConfig{:centered}(
    25.0 * u"mm",                # Focal length
    3.45 * u"μm" / 1pixel,       # Physical pixel size
    4096 * 1pixel,               # Image width in pixels
    3000 * 1pixel,               # Image height in pixels
    0.0 * 1pixel,                # Principal point x-coordinate (centered)
    0.0 * 1pixel                 # Principal point y-coordinate (centered)
)

const CAMERA_CONFIG_OFFSET = CameraConfig{:offset}(
    25.0 * u"mm",                    # Focal length
    3.45 * u"μm" / 1pixel,          # Physical pixel size
    4096 * 1pixel,               # Image width in pixels
    3000 * 1pixel,               # Image height in pixels
    2047.5 * 1pixel,             # Principal point x-coordinate (image center)
    1499.5 * 1pixel              # Principal point y-coordinate (image center)
)

# Backward compatibility
const CAMERA_CONFIG = CAMERA_CONFIG_OFFSET

"""
    project(cam_pos::WorldPoint, cam_rot::RotZYX, world_pt::WorldPoint, 
            config::CameraConfig{S}) -> ProjectionPoint{T, S}

Project a 3D world point to 2D image coordinates using pinhole camera model.

# Arguments
- `cam_pos::WorldPoint`: Camera position in world coordinates
- `cam_rot::RotZYX`: Camera orientation (ZYX Euler angles)
- `world_pt::WorldPoint`: 3D point to project in world coordinates
- `config::CameraConfig{S}`: Camera configuration with coordinate system type

# Returns
- `ProjectionPoint{T, S}`: 2D image coordinates in pixels with matching coordinate system

# Coordinate Systems
For `:centered` coordinates:
- Origin at image center
- X-axis: Left (positive follows cross-track left convention)
- Y-axis: Up (positive follows height up convention)

For `:offset` coordinates:
- Origin at top-left corner
- X-axis: Right (positive to the right)
- Y-axis: Down (positive downward)

# Algorithm
1. Transform world point to camera coordinates
2. Apply pinhole projection model
3. Convert to appropriate coordinate system

# Exceptions
- `DivideError`: If point is at or behind the camera (X ≤ 0)
- `DomainError`: If projection results in invalid coordinates

# Examples
```julia
# Project to centered coordinates
cam_pos = WorldPoint(-500.0u"m", 0.0u"m", 100.0u"m")
cam_rot = RotZYX(0.0, 0.1, 0.0)
runway_corner = WorldPoint(0.0u"m", 25.0u"m", 0.0u"m")

centered_coords = project(cam_pos, cam_rot, runway_corner, CAMERA_CONFIG_CENTERED)
offset_coords = project(cam_pos, cam_rot, runway_corner, CAMERA_CONFIG_OFFSET)
```
"""
function project(
        cam_pos::WorldPoint{T}, cam_rot::RotZYX, world_pt::WorldPoint{T′},
        camconfig::CameraConfig{S}
    ) where {T, T′, S}
    cam_pt = world_pt_to_cam_pt(cam_pos, cam_rot, world_pt)
    cam_pt.x <= 0m && throw(BehindCameraException(cam_pt.x))

    # Calculate focal length in pixels
    (; focal_length, pixel_size) = camconfig
    f_pixels = focal_length / pixel_size

    u_centered = f_pixels * (cam_pt.y / cam_pt.x) |> _uconvert(pixel)  # Left positive
    v_centered = f_pixels * (cam_pt.z / cam_pt.x) |> _uconvert(pixel)  # Up positive

    T′′ = typeof(u_centered)
    return @match camconfig begin
        # points left and up
        ::CameraConfig{:centered} => let
            ProjectionPoint{T′′, :centered}(u_centered, v_centered)
        end
        # points right and down
        ::CameraConfig{:offset} => let
            u = -u_centered + camconfig.optical_center_u
            v = -v_centered + camconfig.optical_center_v
            ProjectionPoint{T′′, :offset}(u, v)
        end

    end
end

# Backward compatibility method
function project(
        cam_pos::WorldPoint{T}, cam_rot::RotZYX, world_pt::WorldPoint{T′};
        config = CAMERA_CONFIG
    ) where {T, T′}
    return project(cam_pos, cam_rot, world_pt, config)
end

"""
    get_focal_length_pixels(config::CameraConfig) -> Quantity

Get focal length in pixels from camera configuration.

# Arguments
- `config::CameraConfig`: Camera configuration parameters

# Returns
- Focal length in pixel units

# Examples
```julia
f_px = get_focal_length_pixels(CAMERA_CONFIG_CENTERED)
println("Focal length: ", f_px)
```
"""
function get_focal_length_pixels(config::CameraConfig)
    return config.focal_length / config.pixel_size
end

