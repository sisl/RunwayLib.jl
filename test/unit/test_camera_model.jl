using Test
using RunwayPoseEstimation

@testset "Camera Model" begin
    @testset "Camera Parameters" begin
        # Test default camera configuration exists
        @test isdefined(RunwayPoseEstimation, :CAMERA_CONFIG)
        
        # Test camera parameters have expected fields
        config = RunwayPoseEstimation.CAMERA_CONFIG
        @test haskey(config, :focal_length_m)
        @test haskey(config, :pixel_size_m)
        @test haskey(config, :image_width_px)
        @test haskey(config, :image_height_px)
        @test haskey(config, :optical_center_u_px)
        @test haskey(config, :optical_center_v_px)
        
        # Test parameter values are reasonable
        @test config.focal_length_m > 0
        @test config.pixel_size_m > 0
        @test config.image_width_px > 0
        @test config.image_height_px > 0
    end
    
    @testset "Projection Functions" begin
        # Test that projection function exists and works
        cam_pos = WorldPoint(-1000.0, 0.0, 100.0)
        cam_rot = RotZYX(0.0, 0.0, 0.0)
        world_pt = WorldPoint(0.0, 0.0, 0.0)
        
        proj_pt = project(cam_pos, cam_rot, world_pt)
        @test isa(proj_pt, ProjectionPoint)
        @test isfinite(proj_pt.x)
        @test isfinite(proj_pt.y)
        
        # Test projection of point directly in front of camera
        world_pt_front = WorldPoint(-999.0, 0.0, 100.0)  # 1m in front
        proj_front = project(cam_pos, cam_rot, world_pt_front)
        
        # Should project near image center
        @test abs(proj_front.x) < 100  # Within 100 pixels of center
        @test abs(proj_front.y) < 100
    end
    
    @testset "Camera Calibration" begin
        # Test focal length conversion from physical to pixel units
        focal_length_m = 25e-3  # 25mm
        pixel_size_m = 0.00345e-3  # Typical pixel size
        focal_length_px = focal_length_m / pixel_size_m
        
        @test focal_length_px > 1000  # Should be reasonable number of pixels
        @test focal_length_px < 100000  # But not unreasonably large
        
        # Test optical center is reasonable
        config = RunwayPoseEstimation.CAMERA_CONFIG
        @test config.optical_center_u_px ≈ config.image_width_px / 2 atol=10
        @test config.optical_center_v_px ≈ config.image_height_px / 2 atol=10
    end
    
    @testset "Inverse Projection" begin
        # Test that we can implement basic inverse projection concepts
        # (This would test actual inverse projection functions when implemented)
        
        # For now, test that projection is invertible in principle
        cam_pos = WorldPoint(-1000.0, 0.0, 100.0)
        cam_rot = RotZYX(0.0, 0.0, 0.0)
        
        # Project a point
        world_pt = WorldPoint(0.0, 10.0, 0.0)
        proj_pt = project(cam_pos, cam_rot, world_pt)
        
        # The projection should be deterministic
        proj_pt2 = project(cam_pos, cam_rot, world_pt)
        @test proj_pt.x ≈ proj_pt2.x
        @test proj_pt.y ≈ proj_pt2.y
    end
end
