"""
Basic tests for the poseest Python package.
"""

import pytest
import sys
import os
from pathlib import Path

# Add the parent directory to the path so we can import poseest
sys.path.insert(0, str(Path(__file__).parent.parent))

import poseest


def test_data_types():
    """Test basic data type construction and conversion."""
    # Test WorldPoint
    wp = poseest.WorldPoint(100.0, -25.0, 50.0)
    assert wp.x == 100.0
    assert wp.y == -25.0
    assert wp.z == 50.0
    
    # Test ProjectionPoint
    pp = poseest.ProjectionPoint(320.5, 240.2)
    assert pp.x == 320.5
    assert pp.y == 240.2
    
    # Test Rotation
    rot = poseest.Rotation(0.1, 0.05, -0.02)
    assert rot.yaw == 0.1
    assert rot.pitch == 0.05
    assert rot.roll == -0.02


def test_c_struct_conversion():
    """Test conversion to/from C structures."""
    # Test WorldPoint conversion
    wp = poseest.WorldPoint(-500.0, 10.0, 80.0)
    wp_c = wp.to_c_struct()
    wp_back = poseest.WorldPoint.from_c_struct(wp_c)
    
    assert wp_back.x == wp.x
    assert wp_back.y == wp.y
    assert wp_back.z == wp.z
    
    # Test ProjectionPoint conversion
    pp = poseest.ProjectionPoint(245.3, 180.7)
    pp_c = pp.to_c_struct()
    pp_back = poseest.ProjectionPoint.from_c_struct(pp_c)
    
    assert pp_back.x == pp.x
    assert pp_back.y == pp.y
    
    # Test Rotation conversion  
    rot = poseest.Rotation(0.05, 0.04, 0.03)
    rot_c = rot.to_c_struct()
    rot_back = poseest.Rotation.from_c_struct(rot_c)
    
    assert rot_back.yaw == rot.yaw
    assert rot_back.pitch == rot.pitch
    assert rot_back.roll == rot.roll


def test_camera_config_enum():
    """Test CameraConfig enum values."""
    assert poseest.CameraConfig.CENTERED == 0
    assert poseest.CameraConfig.OFFSET == 1


def test_estimate_pose_6dof():
    """Test 6DOF pose estimation with synthetic data."""
    # Define runway corners (similar to the test data in Julia)
    runway_corners = [
        poseest.WorldPoint(0.0, -50.0, 0.0),
        poseest.WorldPoint(0.0, 50.0, 0.0), 
        poseest.WorldPoint(3000.0, 50.0, 0.0),
        poseest.WorldPoint(3000.0, -50.0, 0.0),
    ]
    
    # Synthetic projections (roughly what we'd expect from the test case)
    projections = [
        poseest.ProjectionPoint(2000.0, 1500.0),  # Rough estimates
        poseest.ProjectionPoint(2100.0, 1500.0),
        poseest.ProjectionPoint(2100.0, 1600.0),
        poseest.ProjectionPoint(2000.0, 1600.0),
    ]
    
    # Estimate pose
    pose = poseest.estimate_pose_6dof(
        runway_corners,
        projections,
        poseest.CameraConfig.OFFSET
    )
    
    # Basic sanity checks
    assert isinstance(pose, poseest.PoseEstimate)
    assert isinstance(pose.position, poseest.WorldPoint)
    assert isinstance(pose.rotation, poseest.Rotation)
    assert isinstance(pose.residual, float)
    assert isinstance(pose.converged, bool)
    
    # The position should be reasonable (aircraft somewhere above/near runway)
    assert -3000.0 < pose.position.x < 3000.0  # Along runway
    assert -500.0 < pose.position.y < 500.0    # Not too far off centerline
    assert 0.0 < pose.position.z < 1000.0      # Reasonable altitude


def test_insufficient_points_error():
    """Test that insufficient points raises the right error."""
    runway_corners = [
        poseest.WorldPoint(0.0, -50.0, 0.0),
        poseest.WorldPoint(0.0, 50.0, 0.0),  # Only 2 points
    ]
    
    projections = [
        poseest.ProjectionPoint(2000.0, 1500.0),
        poseest.ProjectionPoint(2100.0, 1500.0),
    ]
    
    with pytest.raises(poseest.InsufficientPointsError):
        poseest.estimate_pose_6dof(runway_corners, projections)


def test_mismatched_arrays_error():
    """Test that mismatched array sizes raise the right error."""
    runway_corners = [
        poseest.WorldPoint(0.0, -50.0, 0.0),
        poseest.WorldPoint(0.0, 50.0, 0.0),
        poseest.WorldPoint(3000.0, 50.0, 0.0),
        poseest.WorldPoint(3000.0, -50.0, 0.0),
    ]
    
    projections = [
        poseest.ProjectionPoint(2000.0, 1500.0),  # Only 1 projection
    ]
    
    with pytest.raises(poseest.InvalidInputError):
        poseest.estimate_pose_6dof(runway_corners, projections)


def test_project_point():
    """Test point projection functionality."""
    # Camera at origin looking forward
    camera_pos = poseest.WorldPoint(0.0, 0.0, 0.0)
    camera_rot = poseest.Rotation(0.0, 0.0, 0.0)  # No rotation
    
    # Point in front of camera
    world_point = poseest.WorldPoint(1000.0, 100.0, 50.0)
    
    # Project the point
    projection = poseest.project_point(
        camera_pos,
        camera_rot,
        world_point,
        poseest.CameraConfig.OFFSET
    )
    
    # Basic sanity checks
    assert isinstance(projection, poseest.ProjectionPoint)
    assert isinstance(projection.x, float)
    assert isinstance(projection.y, float)
    
    # Projected coordinates should be reasonable
    assert 0.0 < projection.x < 4096.0  # Within image width
    assert 0.0 < projection.y < 3000.0  # Within image height


if __name__ == "__main__":
    # Run the tests if executed directly
    test_data_types()
    test_c_struct_conversion()
    test_camera_config_enum()
    
    print("Basic tests passed!")
    
    # Try the actual library functions (these require the native library)
    try:
        test_estimate_pose_6dof()
        test_project_point()
        print("Library integration tests passed!")
    except Exception as e:
        print(f"Library tests failed (this is expected if native library not bundled): {e}")
    
    print("All available tests completed!")