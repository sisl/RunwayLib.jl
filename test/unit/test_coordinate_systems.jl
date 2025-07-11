using Test
using RunwayPoseEstimation
using StaticArrays
using Rotations
using LinearAlgebra

@testset "Coordinate Systems" begin
    @testset "WorldPoint" begin
        # Test construction
        wp = WorldPoint(1.0, 2.0, 3.0)
        @test wp.x == 1.0
        @test wp.y == 2.0
        @test wp.z == 3.0
        
        # Test FieldVector interface
        @test length(wp) == 3
        @test wp[1] == 1.0
        @test wp[2] == 2.0
        @test wp[3] == 3.0
        
        # Test arithmetic operations
        wp2 = WorldPoint(4.0, 5.0, 6.0)
        wp_sum = wp + wp2
        @test wp_sum == WorldPoint(5.0, 7.0, 9.0)
        
        # Test scalar multiplication
        wp_scaled = 2.0 * wp
        @test wp_scaled == WorldPoint(2.0, 4.0, 6.0)
    end
    
    @testset "CameraPoint" begin
        # Test construction
        cp = CameraPoint(1.0, 2.0, 3.0)
        @test cp.x == 1.0
        @test cp.y == 2.0
        @test cp.z == 3.0
        
        # Test FieldVector interface
        @test length(cp) == 3
        @test cp[1] == 1.0
        
        # Test arithmetic operations
        cp2 = CameraPoint(4.0, 5.0, 6.0)
        cp_sum = cp + cp2
        @test cp_sum == CameraPoint(5.0, 7.0, 9.0)
    end
    
    @testset "ProjectionPoint" begin
        # Test construction
        pp = ProjectionPoint(100.0, 200.0)
        @test pp.x == 100.0
        @test pp.y == 200.0
        
        # Test FieldVector interface
        @test length(pp) == 2
        @test pp[1] == 100.0
        @test pp[2] == 200.0
        
        # Test arithmetic operations
        pp2 = ProjectionPoint(50.0, 75.0)
        pp_sum = pp + pp2
        @test pp_sum == ProjectionPoint(150.0, 275.0)
    end
    
    @testset "Coordinate Transformations" begin
        # Test world to camera transformation
        cam_pos = WorldPoint(0.0, 0.0, 0.0)
        cam_rot = RotZYX(0.0, 0.0, 0.0)  # Identity rotation
        world_pt = WorldPoint(1.0, 2.0, 3.0)
        
        # With identity transformation, camera point should equal world point
        cam_pt = world_pt_to_cam_pt(cam_pos, cam_rot, world_pt)
        @test cam_pt ≈ CameraPoint(1.0, 2.0, 3.0)
        
        # Test inverse transformation
        world_pt_back = cam_pt_to_world_pt(cam_pos, cam_rot, cam_pt)
        @test world_pt_back ≈ world_pt
        
        # Test with non-trivial transformation
        cam_pos = WorldPoint(10.0, 20.0, 30.0)
        cam_rot = RotZYX(0.0, 0.0, π/2)  # 90 degree yaw rotation
        world_pt = WorldPoint(11.0, 20.0, 30.0)  # 1 unit in x from camera
        
        cam_pt = world_pt_to_cam_pt(cam_pos, cam_rot, world_pt)
        # After 90 degree yaw rotation, x becomes -y in camera frame
        @test cam_pt.y ≈ -1.0 atol=1e-10
        @test abs(cam_pt.x) < 1e-10
        @test abs(cam_pt.z) < 1e-10
    end
    
    @testset "Camera Projection" begin
        # Test basic projection
        cam_pos = WorldPoint(0.0, 0.0, 0.0)
        cam_rot = RotZYX(0.0, 0.0, 0.0)
        world_pt = WorldPoint(1.0, 0.1, 0.1)  # Point in front of camera
        
        proj_pt = project(cam_pos, cam_rot, world_pt)
        @test isa(proj_pt, ProjectionPoint)
        
        # Test that points at camera position cannot be projected (division by zero)
        world_pt_at_camera = WorldPoint(0.0, 0.0, 0.0)
        @test_throws DivideError project(cam_pos, cam_rot, world_pt_at_camera)
        
        # Test projection consistency - points further away should project closer to center
        world_pt_near = WorldPoint(1.0, 0.1, 0.0)
        world_pt_far = WorldPoint(10.0, 1.0, 0.0)  # Same angle but further
        
        proj_near = project(cam_pos, cam_rot, world_pt_near)
        proj_far = project(cam_pos, cam_rot, world_pt_far)
        
        # Both should have same projection (similar triangles)
        @test proj_near.x ≈ proj_far.x atol=1e-6
        @test proj_near.y ≈ proj_far.y atol=1e-6
    end
end
