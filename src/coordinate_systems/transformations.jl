"""
Coordinate system transformations for runway pose estimation.

This module provides functions to transform points between different coordinate systems:
- World to camera coordinates
- Camera to world coordinates
- Camera to image projection coordinates
"""

using LinearAlgebra
using Rotations
using Unitful

"""
    world_pt_to_cam_pt(cam_pos::WorldPoint, cam_rot::RotZYX, world_pt::WorldPoint) -> CameraPoint

Transform a point from world coordinates to camera coordinates.

# Arguments
- `cam_pos::WorldPoint`: Camera position in world coordinates
- `cam_rot::RotZYX`: Camera orientation (ZYX Euler angles: yaw, pitch, roll)
- `world_pt::WorldPoint`: Point to transform in world coordinates

# Returns
- `CameraPoint`: Point in camera coordinate system

# Algorithm
1. Translate point relative to camera position
2. Rotate by inverse of camera rotation to get camera-relative coordinates

# Examples
```julia
# Camera at origin with no rotation
cam_pos = WorldPoint(0.0u"m", 0.0u"m", 0.0u"m")
cam_rot = RotZYX(0.0, 0.0, 0.0)  # No rotation
world_pt = WorldPoint(1.0u"m", 2.0u"m", 3.0u"m")

cam_pt = world_pt_to_cam_pt(cam_pos, cam_rot, world_pt)
# Result: CameraPoint(1.0u"m", 2.0u"m", 3.0u"m")

# Camera with 90-degree yaw rotation
cam_rot = RotZYX(π/2, 0.0, 0.0)  # 90-degree yaw
world_pt = WorldPoint(1.0u"m", 0.0u"m", 0.0u"m")

cam_pt = world_pt_to_cam_pt(cam_pos, cam_rot, world_pt)
# After rotation, x becomes -y in camera frame
```
"""
function world_pt_to_cam_pt(cam_pos::WorldPoint, cam_rot::RotZYX, world_pt::WorldPoint)
    # Translate to camera-relative coordinates
    relative_pt = world_pt - cam_pos

    # Apply inverse rotation (transpose of rotation matrix)
    # This transforms from world frame to camera frame
    cam_vec = cam_rot' * relative_pt

    return CameraPoint(cam_vec)
end

"""
    cam_pt_to_world_pt(cam_pos::WorldPoint, cam_rot::RotZYX, cam_pt::CameraPoint) -> WorldPoint

Transform a point from camera coordinates to world coordinates.

# Arguments
- `cam_pos::WorldPoint`: Camera position in world coordinates
- `cam_rot::RotZYX`: Camera orientation (ZYX Euler angles: yaw, pitch, roll)
- `cam_pt::CameraPoint`: Point to transform in camera coordinates

# Returns
- `WorldPoint`: Point in world coordinate system

# Algorithm
1. Rotate point by camera rotation to get world-relative coordinates
2. Translate by camera position to get absolute world coordinates

# Examples
```julia
# Transform camera point back to world coordinates
cam_pos = WorldPoint(10.0u"m", 20.0u"m", 30.0u"m")
cam_rot = RotZYX(0.0, 0.0, 0.0)  # No rotation
cam_pt = CameraPoint(1.0u"m", 2.0u"m", 3.0u"m")

world_pt = cam_pt_to_world_pt(cam_pos, cam_rot, cam_pt)
# Result: WorldPoint(11.0u"m", 22.0u"m", 33.0u"m")
```
"""
function cam_pt_to_world_pt(cam_pos::WorldPoint, cam_rot::RotZYX, cam_pt::CameraPoint)
    # Convert to SVector for rotation
    cam_vec = SVector(cam_pt.x, cam_pt.y, cam_pt.z)

    # Apply rotation to transform from camera frame to world frame
    world_vec = cam_rot * cam_vec

    # Create world point and translate by camera position
    relative_pt = WorldPoint(world_vec[1], world_vec[2], world_vec[3])

    return cam_pos + relative_pt
end
