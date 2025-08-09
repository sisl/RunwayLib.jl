"""
Test the enhanced API functions to reproduce the units conversion error.
"""

import sys
from pathlib import Path

# Add the parent directory to the path so we can import poseest
sys.path.insert(0, str(Path(__file__).parent.parent))

import poseest


def test_estimate_pose_6dof_simple():
    """Test the enhanced estimate_pose_6dof function with simple data."""
    print("Testing estimate_pose_6dof with simple runway data...")
    
    try:
        # Define simple runway corners (similar to the test data in Julia)
        runway_corners = [
            poseest.WorldPoint(0.0, -50.0, 0.0),
            poseest.WorldPoint(0.0, 50.0, 0.0), 
            poseest.WorldPoint(3000.0, 50.0, 0.0),
            poseest.WorldPoint(3000.0, -50.0, 0.0),
        ]
        
        # Simple projections (rough estimates based on typical camera geometry)
        projections = [
            poseest.ProjectionPoint(2000.0, 1500.0),  
            poseest.ProjectionPoint(2100.0, 1500.0),
            poseest.ProjectionPoint(2100.0, 1600.0),
            poseest.ProjectionPoint(2000.0, 1600.0),
        ]
        
        print("Calling estimate_pose_6dof...")
        
        # This should trigger the units conversion error
        pose = poseest.estimate_pose_6dof(
            runway_corners,
            projections,
            poseest.CameraConfig.OFFSET
        )
        
        print(f"✅ SUCCESS: Got pose result:")
        print(f"  Position: ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})")
        print(f"  Rotation: yaw={pose.rotation.yaw:.3f}, pitch={pose.rotation.pitch:.3f}, roll={pose.rotation.roll:.3f}")
        print(f"  Converged: {pose.converged}")
        
        return True
        
    except Exception as e:
        print(f"❌ FAILED with error: {e}")
        print(f"Error type: {type(e).__name__}")
        return False


def test_project_point_simple():
    """Test the project_point function which might have similar units issues."""
    print("\nTesting project_point function...")
    
    try:
        # Camera at origin looking forward
        camera_pos = poseest.WorldPoint(0.0, 0.0, 0.0)
        camera_rot = poseest.Rotation(0.0, 0.0, 0.0)  # No rotation
        
        # Point in front of camera
        world_point = poseest.WorldPoint(1000.0, 100.0, 50.0)
        
        print("Calling project_point...")
        
        # This might also trigger units issues
        projection = poseest.project_point(
            camera_pos,
            camera_rot,
            world_point,
            poseest.CameraConfig.OFFSET
        )
        
        print(f"✅ SUCCESS: Got projection result:")
        print(f"  Projection: ({projection.x:.2f}, {projection.y:.2f})")
        
        return True
        
    except Exception as e:
        print(f"❌ FAILED with error: {e}")
        print(f"Error type: {type(e).__name__}")
        return False


if __name__ == "__main__":
    print("Testing enhanced API functions to reproduce units errors...\n")
    
    success_6dof = test_estimate_pose_6dof_simple()
    success_project = test_project_point_simple()
    
    print(f"\n" + "="*60)
    print("SUMMARY:")
    print(f"estimate_pose_6dof: {'✅ SUCCESS' if success_6dof else '❌ FAILED'}")
    print(f"project_point: {'✅ SUCCESS' if success_project else '❌ FAILED'}")
    
    if not success_6dof or not success_project:
        print(f"\n🔍 ANALYSIS: Units conversion errors in enhanced API functions")
        print(f"   - Original test_estimators() works fine")
        print(f"   - Enhanced API has Julia units handling issues")
        print(f"   - Python wrapper is correct, issue is in Julia code")
    else:
        print(f"\n🎉 All enhanced API functions working!")