"""
Core functionality for poseest Python package.

This module provides the main Python interface to the Julia/C pose estimation library.
"""

import ctypes
import os
from enum import IntEnum
from typing import List, Union, Tuple
from dataclasses import dataclass

from .native import load_poseest_library


# ============================================================================
# Exception Classes
# ============================================================================

class PoseEstimationError(Exception):
    """Base exception for pose estimation errors."""
    pass


class InvalidInputError(PoseEstimationError):
    """Raised when input parameters are invalid."""
    pass


class BehindCameraError(PoseEstimationError):
    """Raised when a point projects behind the camera."""
    pass


class ConvergenceError(PoseEstimationError):
    """Raised when optimization fails to converge."""
    pass


class InsufficientPointsError(PoseEstimationError):
    """Raised when insufficient points are provided for estimation."""
    pass


# ============================================================================
# C Structure Definitions (matching the C API)
# ============================================================================

class WorldPointF64(ctypes.Structure):
    """C structure for 3D world points."""
    _fields_ = [
        ("x", ctypes.c_double),
        ("y", ctypes.c_double), 
        ("z", ctypes.c_double),
    ]


class ProjectionPointF64(ctypes.Structure):
    """C structure for 2D image projections."""
    _fields_ = [
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
    ]


class RotYPRF64(ctypes.Structure):
    """C structure for rotations (ZYX Euler angles as 3-element vector)."""
    _fields_ = [
        ("data", ctypes.c_double * 3),  # yaw, pitch, roll as array
    ]


class PoseEstimate_C(ctypes.Structure):
    """C structure for pose estimation results."""
    _fields_ = [
        ("position", WorldPointF64),
        ("rotation", RotYPRF64),
        ("residual_norm", ctypes.c_double),
        ("converged", ctypes.c_int),
    ]


# ============================================================================
# Enums and Constants
# ============================================================================

class CameraConfig(IntEnum):
    """Camera configuration options."""
    CENTERED = 0  # Origin at image center, Y-axis up
    OFFSET = 1    # Origin at top-left, Y-axis down


# Error codes (matching C API)
POSEEST_SUCCESS = 0
POSEEST_ERROR_INVALID_INPUT = -1
POSEEST_ERROR_BEHIND_CAMERA = -2
POSEEST_ERROR_NO_CONVERGENCE = -3
POSEEST_ERROR_INSUFFICIENT_POINTS = -4


# ============================================================================
# Python Data Classes
# ============================================================================

@dataclass
class WorldPoint:
    """
    3D point in world coordinate system.
    
    Attributes:
        x: Along-track distance (meters, positive towards far end of runway)
        y: Cross-track distance (meters, positive towards right side of runway)  
        z: Height above runway surface (meters, positive upward)
    """
    x: float
    y: float
    z: float
    
    def to_c_struct(self) -> WorldPointF64:
        """Convert to C structure."""
        return WorldPointF64(self.x, self.y, self.z)
    
    @classmethod
    def from_c_struct(cls, c_struct: WorldPointF64) -> 'WorldPoint':
        """Create from C structure."""
        return cls(c_struct.x, c_struct.y, c_struct.z)


@dataclass
class ProjectionPoint:
    """
    2D point in image coordinate system.
    
    Attributes:
        x: Horizontal image coordinate (pixels)
        y: Vertical image coordinate (pixels)
        
    Note: Coordinate system depends on camera configuration:
    - OFFSET: Origin at top-left, Y-axis down
    - CENTERED: Origin at center, Y-axis up
    """
    x: float
    y: float
    
    def to_c_struct(self) -> ProjectionPointF64:
        """Convert to C structure."""
        return ProjectionPointF64(self.x, self.y)
    
    @classmethod
    def from_c_struct(cls, c_struct: ProjectionPointF64) -> 'ProjectionPoint':
        """Create from C structure."""
        return cls(c_struct.x, c_struct.y)


@dataclass
class Rotation:
    """
    Rotation as ZYX Euler angles.
    
    Attributes:
        yaw: Rotation around Z-axis (radians)
        pitch: Rotation around Y-axis (radians)
        roll: Rotation around X-axis (radians)
    """
    yaw: float
    pitch: float
    roll: float
    
    def to_c_struct(self) -> RotYPRF64:
        """Convert to C structure."""
        c_struct = RotYPRF64()
        c_struct.data[0] = self.yaw
        c_struct.data[1] = self.pitch
        c_struct.data[2] = self.roll
        return c_struct
    
    @classmethod
    def from_c_struct(cls, c_struct: RotYPRF64) -> 'Rotation':
        """Create from C structure."""
        return cls(c_struct.data[0], c_struct.data[1], c_struct.data[2])


@dataclass
class PoseEstimate:
    """
    Complete pose estimation result.
    
    Attributes:
        position: Estimated aircraft position in world coordinates
        rotation: Estimated aircraft attitude (ZYX Euler angles)
        residual: Final optimization residual norm
        converged: Whether optimization converged successfully
    """
    position: WorldPoint
    rotation: Rotation
    residual: float
    converged: bool
    
    @classmethod
    def from_c_struct(cls, c_struct: PoseEstimate_C) -> 'PoseEstimate':
        """Create from C structure."""
        return cls(
            position=WorldPoint.from_c_struct(c_struct.position),
            rotation=Rotation.from_c_struct(c_struct.rotation),
            residual=c_struct.residual_norm,
            converged=bool(c_struct.converged)
        )


# ============================================================================
# Library Loading and Function Setup
# ============================================================================

# Global library instance
_poseest_lib = None


def _get_library():
    """Get the loaded poseest library, loading it if necessary."""
    global _poseest_lib
    if _poseest_lib is None:
        _poseest_lib = load_poseest_library()
        _setup_function_signatures(_poseest_lib)
    return _poseest_lib


def _setup_function_signatures(lib):
    """Set up ctypes function signatures for the library."""
    
    # initialize_poseest_library
    lib.initialize_poseest_library.argtypes = [ctypes.c_char_p]
    lib.initialize_poseest_library.restype = ctypes.c_int
    
    # estimate_pose_6dof
    lib.estimate_pose_6dof.argtypes = [
        ctypes.POINTER(WorldPointF64),      # runway_corners
        ctypes.POINTER(ProjectionPointF64), # projections  
        ctypes.c_int,                      # num_points
        ctypes.c_int,                      # camera_config
        ctypes.POINTER(PoseEstimate_C)     # result
    ]
    lib.estimate_pose_6dof.restype = ctypes.c_int
    
    # estimate_pose_3dof  
    lib.estimate_pose_3dof.argtypes = [
        ctypes.POINTER(WorldPointF64),      # runway_corners
        ctypes.POINTER(ProjectionPointF64), # projections
        ctypes.c_int,                      # num_points  
        ctypes.POINTER(RotYPRF64),        # known_rotation
        ctypes.c_int,                      # camera_config
        ctypes.POINTER(PoseEstimate_C)     # result
    ]
    lib.estimate_pose_3dof.restype = ctypes.c_int
    
    # project_point
    lib.project_point.argtypes = [
        ctypes.POINTER(WorldPointF64),      # camera_position
        ctypes.POINTER(RotYPRF64),        # camera_rotation
        ctypes.POINTER(WorldPointF64),      # world_point
        ctypes.c_int,                      # camera_config
        ctypes.POINTER(ProjectionPointF64)  # result
    ]
    lib.project_point.restype = ctypes.c_int


def _handle_error_code(error_code: int) -> None:
    """Convert C error code to appropriate Python exception."""
    if error_code == POSEEST_SUCCESS:
        return
    elif error_code == POSEEST_ERROR_INVALID_INPUT:
        raise InvalidInputError("Invalid input parameters")
    elif error_code == POSEEST_ERROR_BEHIND_CAMERA:
        raise BehindCameraError("Point is behind camera")
    elif error_code == POSEEST_ERROR_NO_CONVERGENCE:
        raise ConvergenceError("Optimization did not converge")
    elif error_code == POSEEST_ERROR_INSUFFICIENT_POINTS:
        raise InsufficientPointsError("Insufficient number of points")
    else:
        raise PoseEstimationError(f"Unknown error code: {error_code}")


# ============================================================================
# Main API Functions
# ============================================================================

def _ensure_initialized():
    """Ensure the library is initialized with proper environment setup."""
    lib = _get_library()
    
    # Get the depot path from environment (set by native loader)
    depot_path = os.environ.get("JULIA_DEPOT_PATH", "")
    depot_path_bytes = depot_path.encode('utf-8') if depot_path else None
    
    # Initialize the library
    error_code = lib.initialize_poseest_library(depot_path_bytes)
    _handle_error_code(error_code)


def estimate_pose_6dof(
    runway_corners: List[WorldPoint],
    projections: List[ProjectionPoint], 
    camera_config: CameraConfig = CameraConfig.OFFSET
) -> PoseEstimate:
    """
    Estimate 6DOF pose (position + orientation) from runway corner projections.
    
    Args:
        runway_corners: List of at least 4 runway corners in world coordinates
        projections: List of corresponding image projections  
        camera_config: Camera coordinate system configuration
        
    Returns:
        Estimated pose with position, rotation, and convergence info
        
    Raises:
        InvalidInputError: If inputs are invalid
        InsufficientPointsError: If fewer than 4 points provided
        ConvergenceError: If optimization fails to converge
    """
    if len(runway_corners) != len(projections):
        raise InvalidInputError("Number of corners and projections must match")
    
    if len(runway_corners) < 4:
        raise InsufficientPointsError("At least 4 points required for 6DOF estimation")
    
    # Ensure library is initialized
    _ensure_initialized()
    lib = _get_library()
    
    # Convert to C arrays
    num_points = len(runway_corners)
    corners_array = (WorldPointF64 * num_points)(*[c.to_c_struct() for c in runway_corners])
    projs_array = (ProjectionPointF64 * num_points)(*[p.to_c_struct() for p in projections])
    
    # Prepare result structure
    result = PoseEstimate_C()
    
    # Call C function
    error_code = lib.estimate_pose_6dof(
        corners_array,
        projs_array, 
        num_points,
        int(camera_config),
        ctypes.byref(result)
    )
    
    # Handle errors
    _handle_error_code(error_code)
    
    # Convert result back to Python
    return PoseEstimate.from_c_struct(result)


def estimate_pose_3dof(
    runway_corners: List[WorldPoint],
    projections: List[ProjectionPoint],
    known_rotation: Rotation,
    camera_config: CameraConfig = CameraConfig.OFFSET
) -> PoseEstimate:
    """
    Estimate 3DOF pose (position only) when orientation is known.
    
    Args:
        runway_corners: List of at least 3 runway corners in world coordinates
        projections: List of corresponding image projections
        known_rotation: Known aircraft attitude
        camera_config: Camera coordinate system configuration
        
    Returns:
        Estimated pose with position and known rotation
        
    Raises:
        InvalidInputError: If inputs are invalid
        InsufficientPointsError: If fewer than 3 points provided
        ConvergenceError: If optimization fails to converge
    """
    if len(runway_corners) != len(projections):
        raise InvalidInputError("Number of corners and projections must match")
    
    if len(runway_corners) < 3:
        raise InsufficientPointsError("At least 3 points required for 3DOF estimation")
    
    # Ensure library is initialized
    _ensure_initialized()
    lib = _get_library()
    
    # Convert to C arrays
    num_points = len(runway_corners)
    corners_array = (WorldPointF64 * num_points)(*[c.to_c_struct() for c in runway_corners])
    projs_array = (ProjectionPointF64 * num_points)(*[p.to_c_struct() for p in projections])
    rotation_c = known_rotation.to_c_struct()
    
    # Prepare result structure
    result = PoseEstimate_C()
    
    # Call C function
    error_code = lib.estimate_pose_3dof(
        corners_array,
        projs_array,
        num_points,
        ctypes.byref(rotation_c),
        int(camera_config), 
        ctypes.byref(result)
    )
    
    # Handle errors
    _handle_error_code(error_code)
    
    # Convert result back to Python
    return PoseEstimate.from_c_struct(result)


def project_point(
    camera_position: WorldPoint,
    camera_rotation: Rotation,
    world_point: WorldPoint,
    camera_config: CameraConfig = CameraConfig.OFFSET
) -> ProjectionPoint:
    """
    Project a 3D world point to 2D image coordinates.
    
    Args:
        camera_position: Camera position in world coordinates
        camera_rotation: Camera attitude (ZYX Euler angles)
        world_point: 3D point to project
        camera_config: Camera coordinate system configuration
        
    Returns:
        2D image projection of the world point
        
    Raises:
        InvalidInputError: If inputs are invalid
        BehindCameraError: If point is behind the camera
    """
    # Ensure library is initialized
    _ensure_initialized()
    lib = _get_library()
    
    # Convert to C structures
    cam_pos_c = camera_position.to_c_struct()
    cam_rot_c = camera_rotation.to_c_struct()
    world_pt_c = world_point.to_c_struct()
    
    # Prepare result structure
    result = ProjectionPointF64()
    
    # Call C function
    error_code = lib.project_point(
        ctypes.byref(cam_pos_c),
        ctypes.byref(cam_rot_c),
        ctypes.byref(world_pt_c),
        int(camera_config),
        ctypes.byref(result)
    )
    
    # Handle errors
    _handle_error_code(error_code)
    
    # Convert result back to Python
    return ProjectionPoint.from_c_struct(result)