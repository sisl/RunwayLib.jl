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
        cam_pos::WorldPoint, cam_rot::RotZYX, world_pt::WorldPoint,
        camconfig::CameraConfig{S}
    ) where {S}
    cam_pt = world_pt_to_cam_pt(cam_pos, cam_rot, world_pt)
    cam_pt.x <= 0m && throw(BehindCameraException(cam_pt.x))

    # Calculate focal length in pixels
    (; focal_length, pixel_size) = camconfig
    f_pixels = focal_length / pixel_size

    u_centered = f_pixels * (cam_pt.y / cam_pt.x) |> _uconvert(pixel)  # Left positive
    v_centered = f_pixels * (cam_pt.z / cam_pt.x) |> _uconvert(pixel)  # Up positive

    T = typeof(u_centered)
    return @match camconfig begin
        # points left and up
        ::CameraConfig{:centered} => let
            ProjectionPoint{T, :centered}(u_centered, v_centered)
        end
        # points right and down
        ::CameraConfig{:offset} => let
            u = -u_centered + camconfig.optical_center_u
            v = -v_centered + camconfig.optical_center_v
            ProjectionPoint{T, :offset}(u, v)
        end

    end
end

# Backward compatibility method
function project(
        cam_pos::WorldPoint, cam_rot::RotZYX, world_pt::WorldPoint;
        config = CAMERA_CONFIG
    )
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

# Backward compatibility
get_focal_length_pixels() = get_focal_length_pixels(CAMERA_CONFIG)

"""
    get_field_of_view(config::CameraConfig) -> NamedTuple

Calculate horizontal and vertical field of view angles.

# Arguments
- `config::CameraConfig`: Camera configuration parameters

# Returns
- `NamedTuple` with `horizontal` and `vertical` FOV in radians

# Examples
```julia
fov = get_field_of_view(CAMERA_CONFIG_CENTERED)
println("Horizontal FOV: ", rad2deg(fov.horizontal), " degrees")
println("Vertical FOV: ", rad2deg(fov.vertical), " degrees")
```
"""
function get_field_of_view(config::CameraConfig)
    # Calculate sensor dimensions
    sensor_width = config.image_width * config.pixel_size
    sensor_height = config.image_height * config.pixel_size

    # Calculate field of view angles
    horizontal_fov = 2 * atan(ustrip(sensor_width) / (2 * ustrip(config.focal_length)))
    vertical_fov = 2 * atan(ustrip(sensor_height) / (2 * ustrip(config.focal_length)))

    return (horizontal = horizontal_fov, vertical = vertical_fov)
end

# Backward compatibility
get_field_of_view() = get_field_of_view(CAMERA_CONFIG)

"""
    pixel_to_ray_direction(pixel_pt::ProjectionPoint{T, S}, config::CameraConfig{S}) -> CameraPoint

Convert pixel coordinates to normalized ray direction in camera coordinates.

# Arguments
- `pixel_pt::ProjectionPoint{T, S}`: Pixel coordinates with coordinate system type
- `config::CameraConfig{S}`: Camera configuration with matching coordinate system

# Returns
- `CameraPoint`: Normalized ray direction in camera coordinates

# Algorithm
Converts pixel coordinates back to normalized ray direction using inverse
of the pinhole camera model. The resulting ray has unit length.

# Examples
```julia
# Get ray direction for center pixel
center_pixel = ProjectionPoint{Float64, :offset}(2048.0*1pixel, 1500.0*1pixel)
ray_dir = pixel_to_ray_direction(center_pixel, CAMERA_CONFIG_OFFSET)
println("Ray direction: ", ray_dir)
```
"""
function pixel_to_ray_direction(pixel_pt::ProjectionPoint{T, S}, config::CameraConfig{S}) where {T, S}
    # Extract camera parameters
    focal_length = config.focal_length
    pixel_size = config.pixel_size

    # Calculate focal length in pixels
    f_pixels = focal_length / pixel_size

    if S == :centered
        # For centered coordinates, pixel coordinates are already centered
        u_centered = pixel_pt.x
        v_centered = pixel_pt.y

        # Convert to camera coordinates (flip signs for centered system)
        # In camera frame: X=forward, Y=right, Z=down
        x_cam = 1.0u"m"  # Normalized forward direction
        y_cam = -u_centered / f_pixels * 1.0u"m"  # Right direction (flip left to right)
        z_cam = -v_centered / f_pixels * 1.0u"m"  # Down direction (flip up to down)

    elseif S == :offset
        # For offset coordinates, subtract principal point
        optical_center_u = config.optical_center_u
        optical_center_v = config.optical_center_v

        u_centered = pixel_pt.x - optical_center_u
        v_centered = pixel_pt.y - optical_center_v

        # Convert to camera coordinates
        # In camera frame: X=forward, Y=right, Z=down
        x_cam = 1.0u"m"  # Normalized forward direction
        y_cam = u_centered / f_pixels * 1.0u"m"  # Right direction
        z_cam = v_centered / f_pixels * 1.0u"m"  # Down direction
    else
        error("Unknown coordinate system: $S")
    end

    # Create and normalize ray direction
    ray = CameraPoint(x_cam, y_cam, z_cam)
    ray_magnitude = sqrt(ray.x^2 + ray.y^2 + ray.z^2)

    return CameraPoint(ray.x / ray_magnitude, ray.y / ray_magnitude, ray.z / ray_magnitude)
end

# Backward compatibility method
function pixel_to_ray_direction(pixel_pt::ProjectionPoint, config = CAMERA_CONFIG)
    # Assume old ProjectionPoint is offset type
    pixel_pt_typed = ProjectionPoint{typeof(pixel_pt.x), :offset}(pixel_pt.x, pixel_pt.y)
    config_typed = CameraConfig{:offset}(
        config.focal_length, config.pixel_size,
        config.image_width, config.image_height,
        config.optical_center_u, config.optical_center_v
    )
    return pixel_to_ray_direction(pixel_pt_typed, config_typed)
end
