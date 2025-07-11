using Test
using RunwayPoseEstimation
using Unitful

@testset "Camera Model" begin
    @testset "Camera Parameters" begin
        # Test typical camera parameters with proper units
        focal_length = 25.0u"mm"        # 25mm focal length
        pixel_size = 3.45u"μm"          # 3.45 micron pixel size
        image_width = 4096u"pixel"      # Image width in pixels
        image_height = 3000u"pixel"     # Image height in pixels
        optical_center_u = 2047.5u"pixel"  # Principal point x
        optical_center_v = 1499.5u"pixel"  # Principal point y
        
        @test focal_length > 0.0u"mm"
        @test pixel_size > 0.0u"μm"
        @test image_width > 0u"pixel"
        @test image_height > 0u"pixel"
        @test optical_center_u ≈ image_width / 2 - 0.5u"pixel"
        @test optical_center_v ≈ image_height / 2 - 0.5u"pixel"
        
        # Test unit conversions
        @test uconvert(u"m", focal_length) ≈ 0.025u"m"
        @test uconvert(u"m", pixel_size) ≈ 3.45e-6u"m"
        @test uconvert(u"mm", pixel_size) ≈ 0.00345u"mm"
    end
    
    @testset "Projection Functions" begin
        # Test basic projection geometry with units
        cam_pos = WorldPoint(-1000.0u"m", 0.0u"m", 100.0u"m")
        cam_rot = RotZYX(0.0, 0.0, 0.0)
        world_pt = WorldPoint(0.0u"m", 0.0u"m", 0.0u"m")
        
        proj_pt = project(cam_pos, cam_rot, world_pt)
        @test isa(proj_pt, ProjectionPoint)
        @test isfinite(ustrip(proj_pt.x))
        @test isfinite(ustrip(proj_pt.y))
        
        # Test projection of point directly in front of camera
        world_pt_front = WorldPoint(-999.0u"m", 0.0u"m", 100.0u"m")  # 1m in front
        proj_front = project(cam_pos, cam_rot, world_pt_front)
        
        # Should project near image center
        @test abs(proj_front.x) < 100u"pixel"  # Within 100 pixels of center
        @test abs(proj_front.y) < 100u"pixel"
        
        # Test focal length in pixels
        focal_length = 25.0u"mm"
        pixel_size = 3.45u"μm"
        focal_length_px = focal_length / pixel_size
        @test uconvert(u"pixel", focal_length_px) ≈ 7246.377u"pixel" rtol=1e-3
    end
    
    @testset "Camera Calibration" begin
        # Test focal length conversions with units
        focal_length = 25.0u"mm"
        pixel_size = 3.45u"μm"
        
        focal_length_px = focal_length / pixel_size
        @test uconvert(u"pixel", focal_length_px) ≈ 7246.377u"pixel" rtol=1e-3
        
        # Test field of view calculations
        image_width_px = 4096u"pixel"
        sensor_width = image_width_px * pixel_size
        @test uconvert(u"mm", sensor_width) ≈ 14.1312u"mm" rtol=1e-3
        
        # Calculate horizontal field of view
        sensor_width_m = uconvert(u"m", sensor_width)
        focal_length_m = uconvert(u"m", focal_length)
        horizontal_fov_rad = 2 * atan(ustrip(sensor_width_m) / (2 * ustrip(focal_length_m)))
        horizontal_fov_deg = rad2deg(horizontal_fov_rad)
        
        @test horizontal_fov_deg > 0
        @test horizontal_fov_deg < 180  # Should be reasonable FOV
        @test horizontal_fov_deg ≈ 31.0 rtol=0.1  # Approximately 31 degrees for this setup
        
        # Test typical calibration parameter ranges with units
        focal_lengths = [16.0u"mm", 25.0u"mm", 35.0u"mm", 50.0u"mm", 85.0u"mm"]
        pixel_sizes = [1.4u"μm", 2.2u"μm", 3.45u"μm", 5.5u"μm", 9.0u"μm"]
        
        for f in focal_lengths
            @test f > 0.0u"mm"
            @test f < 200.0u"mm"  # Reasonable range for aircraft cameras
        end
        
        for p in pixel_sizes
            @test p > 0.0u"μm"
            @test p < 20.0u"μm"  # Reasonable range for modern sensors
        end
    end
    
    @testset "Inverse Projection" begin
        # Test inverse projection concepts with units
        cam_pos = WorldPoint(-1000.0u"m", 0.0u"m", 100.0u"m")
        cam_rot = RotZYX(0.0, 0.0, 0.0)
        
        # Project a point
        world_pt = WorldPoint(0.0u"m", 10.0u"m", 0.0u"m")
        proj_pt = project(cam_pos, cam_rot, world_pt)
        
        # The projection should be deterministic
        proj_pt2 = project(cam_pos, cam_rot, world_pt)
        @test proj_pt.x ≈ proj_pt2.x
        @test proj_pt.y ≈ proj_pt2.y
        
        # Test pixel to ray direction concepts
        pixel_point = ProjectionPoint(2048.0u"pixel", 1500.0u"pixel")  # At optical center
        focal_length = 25.0u"mm"
        pixel_size = 3.45u"μm"
        
        # Test f-number calculations
        aperture_diameter = 5.0u"mm"
        f_number = focal_length / aperture_diameter  # f/5 for 25mm lens
        @test f_number ≈ 5.0 rtol=1e-3
    end
end
