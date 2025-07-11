using Test
using RunwayLib
using Unitful
using StaticArrays

@testset "Camera Model" begin
    @testset "Camera Parameters" begin
        # Test typical camera parameters with proper units
        focal_length = 25.0u"mm"        # 25mm focal length
        pixel_size = 3.45u"μm" / pixel          # 3.45 micron pixel size
        image_width = 4096 * pixel      # Image width in pixels
        image_height = 3000 * pixel     # Image height in pixels
        optical_center_u = 2047.5 * pixel  # Principal point x
        optical_center_v = 1499.5 * pixel  # Principal point y

        @test focal_length > 0.0u"mm"
        @test pixel_size > 0.0u"μm" / pixel
        @test image_width > 0 * pixel
        @test image_height > 0 * pixel
        @test optical_center_u ≈ image_width / 2 - 0.5 * pixel
        @test optical_center_v ≈ image_height / 2 - 0.5 * pixel

        # Test unit conversions
        @test uconvert(u"m", focal_length) ≈ 0.025u"m"
        @test uconvert(u"m" / pixel, pixel_size) ≈ 3.45e-6u"m" / pixel
        @test uconvert(u"mm" / pixel, pixel_size) ≈ 0.00345u"mm" / pixel
    end

    @testset "Projection Functions" begin
        # Test basic projection geometry with units
        cam_pos = WorldPoint(-1000.0u"m", 0.0u"m", 100.0u"m")
        cam_rot = RotZYX(0.0, 0.0, 0.0)
        world_pt = WorldPoint(0.0u"m", 0.0u"m", 0.0u"m")

        # Test with both coordinate systems
        proj_pt_offset = project(cam_pos, cam_rot, world_pt, CAMERA_CONFIG_OFFSET)
        proj_pt_centered = project(cam_pos, cam_rot, world_pt, CAMERA_CONFIG_CENTERED)

        @test isa(proj_pt_offset, ProjectionPoint)
        @test isa(proj_pt_centered, ProjectionPoint)
        @test isfinite(ustrip(proj_pt_offset.x))
        @test isfinite(ustrip(proj_pt_offset.y))
        @test isfinite(ustrip(proj_pt_centered.x))
        @test isfinite(ustrip(proj_pt_centered.y))

        # Test projection of point directly in front of camera
        world_pt_front = WorldPoint(-999.0u"m", 0.0u"m", 100.0u"m")  # 1m in front
        proj_front_offset = project(cam_pos, cam_rot, world_pt_front, CAMERA_CONFIG_OFFSET)
        proj_front_centered = project(cam_pos, cam_rot, world_pt_front, CAMERA_CONFIG_CENTERED)

        # For offset coordinates: should project near image center
        @test abs(proj_front_offset.x - CAMERA_CONFIG_OFFSET.optical_center_u) < 100 * pixel
        @test abs(proj_front_offset.y - CAMERA_CONFIG_OFFSET.optical_center_v) < 100 * pixel

        # For centered coordinates: should project near origin
        @test abs(proj_front_centered.x) < 100 * pixel
        @test abs(proj_front_centered.y) < 100 * pixel

        # Test focal length in pixels
        focal_length = 25.0u"mm"
        pixel_size = 3.45u"μm" / pixel
        focal_length_px = focal_length / pixel_size
        @test uconvert(pixel, focal_length_px) ≈ 7246.377 * pixel rtol = 1.0e-3
    end

    @testset "Camera Calibration" begin
        # Test focal length conversions with units
        focal_length = 25.0u"mm"
        pixel_size = 3.45u"μm" / pixel

        focal_length_px = focal_length / pixel_size
        @test uconvert(pixel, focal_length_px) ≈ 7246.377 * pixel rtol = 1.0e-3

        # Test field of view calculations
        image_width_px = 4096 * pixel
        sensor_width = image_width_px * pixel_size
        @test uconvert(u"mm", sensor_width) ≈ 14.1312u"mm" rtol = 1.0e-3

        # Calculate horizontal field of view
        sensor_width_m = uconvert(u"m", sensor_width)
        focal_length_m = uconvert(u"m", focal_length)
        horizontal_fov_rad = 2 * atan(ustrip(sensor_width_m) / (2 * ustrip(focal_length_m)))
        horizontal_fov_deg = rad2deg(horizontal_fov_rad)

        @test horizontal_fov_deg > 0
        @test horizontal_fov_deg < 180  # Should be reasonable FOV
        @test horizontal_fov_deg ≈ 31.0 rtol = 0.1  # Approximately 31 degrees for this setup

        # Test typical calibration parameter ranges with units using StaticArrays
        focal_lengths = SA[16.0u"mm", 25.0u"mm", 35.0u"mm", 50.0u"mm", 85.0u"mm"]
        pixel_sizes = SA[1.4u"μm", 2.2u"μm", 3.45u"μm", 5.5u"μm", 9.0u"μm"]

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
        pixel_point_offset = ProjectionPoint{typeof(1.0pixel), :offset}(2048.0 * pixel, 1500.0 * pixel)
        pixel_point_centered = ProjectionPoint{typeof(1.0pixel), :centered}(0.0 * pixel, 0.0 * pixel)

        ray_dir_offset = pixel_to_ray_direction(pixel_point_offset, CAMERA_CONFIG_OFFSET)
        ray_dir_centered = pixel_to_ray_direction(pixel_point_centered, CAMERA_CONFIG_CENTERED)

        @test isa(ray_dir_offset, CameraPoint)
        @test isa(ray_dir_centered, CameraPoint)

        focal_length = 25.0u"mm"
        pixel_size = 3.45u"μm" / pixel

        # Test f-number calculations
        aperture_diameter = 5.0u"mm"
        f_number = focal_length / aperture_diameter  # f/5 for 25mm lens
        @test f_number ≈ 5.0 rtol = 1.0e-3
    end
end
