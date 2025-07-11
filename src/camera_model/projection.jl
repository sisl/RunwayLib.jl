"""
Camera projection model for runway pose estimation.

This module implements the pinhole camera model for projecting 3D points
to 2D image coordinates, with support for units and realistic camera parameters.
"""

using LinearAlgebra
using Unitful

# Default camera configuration with units
const CAMERA_CONFIG = (
    focal_length=25.0u"mm",           # Focal length
    pixel_size=3.45u"μm" / 1pixel,             # Physical pixel size
    image_width=4096 * 1pixel,         # Image width in pixels
    image_height=3000 * 1pixel,        # Image height in pixels
    optical_center_u=2047.5 * 1pixel,  # Principal point x-coordinate
    optical_center_v=1499.5 * 1pixel   # Principal point y-coordinate
)

"""
    project(cam_pos::WorldPoint, cam_rot::RotZYX, world_pt::WorldPoint; 
            config=CAMERA_CONFIG) -> ProjectionPoint

Project a 3D world point to 2D image coordinates using pinhole camera model.

# Arguments
- `cam_pos::WorldPoint`: Camera position in world coordinates
- `cam_rot::RotZYX`: Camera orientation (ZYX Euler angles)
- `world_pt::WorldPoint`: 3D point to project in world coordinates
- `config`: Camera configuration parameters (optional)

# Returns
- `ProjectionPoint`: 2D image coordinates in pixels

# Algorithm
1. Transform world point to camera coordinates
2. Apply pinhole projection model
3. Convert to pixel coordinates using camera intrinsics

# Camera Model
The pinhole camera model projects 3D points (X, Y, Z) in camera coordinates
to 2D image points (u, v) using:

```
u = f_x * (X/Z) + c_x
v = f_y * (Y/Z) + c_y
```

where:
- `f_x, f_y`: Focal lengths in pixels
- `c_x, c_y`: Principal point coordinates
- `Z > 0`: Point must be in front of camera

# Exceptions
- `DivideError`: If point is at or behind the camera (Z ≤ 0)
- `DomainError`: If projection results in invalid coordinates

# Examples
```julia
# Project runway corner to image
cam_pos = WorldPoint(-500.0u"m", 0.0u"m", 100.0u"m")
cam_rot = RotZYX(0.0, 0.1, 0.0)  # Slight pitch down
runway_corner = WorldPoint(0.0u"m", 25.0u"m", 0.0u"m")

pixel_coords = project(cam_pos, cam_rot, runway_corner)
println("Pixel coordinates: (", pixel_coords.x, ", ", pixel_coords.y, ")")
```
"""
function project(cam_pos::WorldPoint, cam_rot::RotZYX, world_pt::WorldPoint;
    config=CAMERA_CONFIG)
    # Transform to camera coordinates
    cam_pt = world_pt_to_cam_pt(cam_pos, cam_rot, world_pt)

    # Check if point is in front of camera
    if ustrip(cam_pt.x) <= 0
        throw(DivideError())
    end

    # Extract camera parameters
    focal_length = config.focal_length
    pixel_size = config.pixel_size
    optical_center_u = config.optical_center_u
    optical_center_v = config.optical_center_v

    # Calculate focal length in pixels
    f_pixels = uconvert(pixel, focal_length / pixel_size)

    # Apply pinhole projection model
    # Note: In camera coordinates, X is forward, Y is right, Z is down
    # Standard projection: u = f * (Y/X), v = f * (Z/X)
    u_centered = f_pixels * (cam_pt.y / cam_pt.x)
    v_centered = f_pixels * (cam_pt.z / cam_pt.x)

    # Convert to image coordinates (add principal point)
    u = u_centered + optical_center_u
    v = v_centered + optical_center_v

    # Check for valid pixel coordinates
    if !isfinite(ustrip(u)) || !isfinite(ustrip(v))
        throw(DomainError("Invalid projection coordinates"))
    end

    return ProjectionPoint(u, v)
end

"""
    get_focal_length_pixels(config=CAMERA_CONFIG) -> Quantity

Get focal length in pixels from camera configuration.

# Arguments
- `config`: Camera configuration parameters

# Returns
- Focal length in pixel units

# Examples
```julia
f_px = get_focal_length_pixels()
println("Focal length: ", f_px)
```
"""
function get_focal_length_pixels(config=CAMERA_CONFIG)
    return config.focal_length / config.pixel_size
end

"""
    get_field_of_view(config=CAMERA_CONFIG) -> NamedTuple

Calculate horizontal and vertical field of view angles.

# Arguments
- `config`: Camera configuration parameters

# Returns
- `NamedTuple` with `horizontal` and `vertical` FOV in radians

# Examples
```julia
fov = get_field_of_view()
println("Horizontal FOV: ", rad2deg(fov.horizontal), " degrees")
println("Vertical FOV: ", rad2deg(fov.vertical), " degrees")
```
"""
function get_field_of_view(config=CAMERA_CONFIG)
    # Calculate sensor dimensions
    sensor_width = config.image_width * config.pixel_size
    sensor_height = config.image_height * config.pixel_size

    # Calculate field of view angles
    horizontal_fov = 2 * atan(ustrip(sensor_width) / (2 * ustrip(config.focal_length)))
    vertical_fov = 2 * atan(ustrip(sensor_height) / (2 * ustrip(config.focal_length)))

    return (horizontal=horizontal_fov, vertical=vertical_fov)
end

"""
    pixel_to_ray_direction(pixel_pt::ProjectionPoint, config=CAMERA_CONFIG) -> CameraPoint

Convert pixel coordinates to normalized ray direction in camera coordinates.

# Arguments
- `pixel_pt::ProjectionPoint`: Pixel coordinates
- `config`: Camera configuration parameters

# Returns
- `CameraPoint`: Normalized ray direction in camera coordinates

# Algorithm
Converts pixel coordinates back to normalized ray direction using inverse
of the pinhole camera model. The resulting ray has unit length.

# Examples
```julia
# Get ray direction for center pixel
center_pixel = ProjectionPoint(2048.0*1pixel, 1500.0*1pixel)
ray_dir = pixel_to_ray_direction(center_pixel)
println("Ray direction: ", ray_dir)
```
"""
function pixel_to_ray_direction(pixel_pt::ProjectionPoint, config=CAMERA_CONFIG)
    # Extract camera parameters
    focal_length = config.focal_length
    pixel_size = config.pixel_size
    optical_center_u = config.optical_center_u
    optical_center_v = config.optical_center_v

    # Calculate focal length in pixels
    f_pixels = focal_length / pixel_size

    # Convert to centered coordinates
    u_centered = pixel_pt.x - optical_center_u
    v_centered = pixel_pt.y - optical_center_v

    # Convert to normalized camera coordinates
    # In camera frame: X=forward, Y=right, Z=down
    x_cam = 1.0u"m"  # Normalized forward direction
    y_cam = u_centered / f_pixels * 1.0u"m"  # Right direction
    z_cam = v_centered / f_pixels * 1.0u"m"  # Down direction

    # Create and normalize ray direction
    ray = CameraPoint(x_cam, y_cam, z_cam)
    ray_magnitude = sqrt(ray.x^2 + ray.y^2 + ray.z^2)

    return CameraPoint(ray.x / ray_magnitude, ray.y / ray_magnitude, ray.z / ray_magnitude)
end
