using Test
using RunwayLib
using Unitful
using StaticArrays
using Rotations

@testset "Camera Model" begin
    @testset "Projection Functions" begin
        @testset "Perfectly centered" begin
            # Test basic projection geometry with units
            cam_pos = WorldPoint(-1000.0u"m", 0.0u"m", 0.0u"m")
            cam_rot = RotZYX(0.0, 0.0, 0.0)
            world_pt = WorldPoint(0.0u"m", 0.0u"m", 0.0u"m")
            #
            # Test with both coordinate systems
            proj_pt_centered = project(cam_pos, cam_rot, world_pt, CAMERA_CONFIG_CENTERED)
            proj_pt_offset = project(cam_pos, cam_rot, world_pt, CAMERA_CONFIG_OFFSET)
            #
            @test proj_pt_centered ≈ ProjectionPoint(0pixel, 0pixel)
            @test proj_pt_offset ≈ let conf = CAMERA_CONFIG_OFFSET
                ProjectionPoint(conf.optical_center_u, conf.optical_center_v)
            end
        end

        @testset "Check signs: rotated" begin
            # Test basic projection geometry with units
            cam_pos = WorldPoint(-1000.0m, 0.0m, 0.0m)
            # slightly tilted nose up and right
            cam_rot = RotZYX(roll=0°, pitch=0°, yaw=0°)
            world_pt = WorldPoint(0.0m, 10.0m, 10.0m)
            #
            # Test with both coordinate systems
            proj_pt_centered = project(cam_pos, cam_rot, world_pt, CAMERA_CONFIG_CENTERED)
            proj_pt_offset = project(cam_pos, cam_rot, world_pt, CAMERA_CONFIG_OFFSET)
            #
            @test proj_pt_centered.x > 0pixel
            @test proj_pt_centered.y > 0pixel
            #
            @test proj_pt_offset.x < CAMERA_CONFIG_OFFSET.optical_center_u
            @test proj_pt_offset.y < CAMERA_CONFIG_OFFSET.optical_center_v
        end

        @testset "Check signs: roll" begin
            # Test basic projection geometry with units
            cam_pos = WorldPoint(-1000.0m, 0.0m, 0.0m)
            # slightly tilted nose up and right
            cam_rot = RotZYX(roll=90°, pitch=0°, yaw=0°)
            world_pt = WorldPoint(0.0m, 10.0m, 10.0m)
            #
            # Test with both coordinate systems
            proj_pt_centered = project(cam_pos, cam_rot, world_pt, CAMERA_CONFIG_CENTERED)
            proj_pt_offset = project(cam_pos, cam_rot, world_pt, CAMERA_CONFIG_OFFSET)
            #
            @test proj_pt_centered.x > 0pixel
            @test proj_pt_centered.y < 0pixel
            #
            @test proj_pt_offset.x < CAMERA_CONFIG_OFFSET.optical_center_u
            @test proj_pt_offset.y > CAMERA_CONFIG_OFFSET.optical_center_v
        end

        @testset "Check signs: pitch and yaw" begin
            # Test basic projection geometry with units
            cam_pos = WorldPoint(-1000.0m, 0.0m, 0.0m)
            # slightly tilted nose up and right
            cam_rot = RotZYX(roll=0°, pitch=-1°, yaw=-1°)
            world_pt = WorldPoint(0.0m, 0.0m, 0.0m)
            #
            # Test with both coordinate systems
            proj_pt_centered = project(cam_pos, cam_rot, world_pt, CAMERA_CONFIG_CENTERED)
            proj_pt_offset = project(cam_pos, cam_rot, world_pt, CAMERA_CONFIG_OFFSET)
            #
            @test proj_pt_centered.x > 0pixel
            @test proj_pt_centered.y < 0pixel
            #
            @test proj_pt_offset.x < CAMERA_CONFIG_OFFSET.optical_center_u
            @test proj_pt_offset.y > CAMERA_CONFIG_OFFSET.optical_center_v
        end

        @testset "Check throws behind camera" begin
            # Test basic projection geometry with units
            cam_pos = WorldPoint(0.0m, 0.0m, 0.0m)
            # slightly tilted nose up and right
            cam_rot = RotZYX(roll=0°, pitch=-1°, yaw=-1°)
            world_pt = WorldPoint(-1.0m, 0.0m, 0.0m)
            @test_throws BehindCameraException project(cam_pos, cam_rot, world_pt, CAMERA_CONFIG_CENTERED)
            @test_throws BehindCameraException project(cam_pos, cam_rot, world_pt, CAMERA_CONFIG_OFFSET)
        end
    end

    # @testset "Camera Calibration" begin
    #     # Test focal length conversions with units
    #     focal_length = 25.0u"mm"
    #     pixel_size = 3.45u"μm" / pixel

    #     focal_length_px = focal_length / pixel_size
    #     @test uconvert(pixel, focal_length_px) ≈ 7246.377 * pixel rtol = 1.0e-3

    #     # Test field of view calculations
    #     image_width_px = 4096 * pixel
    #     sensor_width = image_width_px * pixel_size
    #     @test uconvert(u"mm", sensor_width) ≈ 14.1312u"mm" rtol = 1.0e-3

    #     # Calculate horizontal field of view
    #     sensor_width_m = uconvert(u"m", sensor_width)
    #     focal_length_m = uconvert(u"m", focal_length)
    #     horizontal_fov_rad = 2 * atan(ustrip(sensor_width_m) / (2 * ustrip(focal_length_m)))
    #     horizontal_fov_deg = rad2deg(horizontal_fov_rad)

    #     @test horizontal_fov_deg > 0
    #     @test horizontal_fov_deg < 180  # Should be reasonable FOV
    #     @test horizontal_fov_deg ≈ 31.0 rtol = 0.1  # Approximately 31 degrees for this setup

    #     # Test typical calibration parameter ranges with units using StaticArrays
    #     focal_lengths = SA[16.0u"mm", 25.0u"mm", 35.0u"mm", 50.0u"mm", 85.0u"mm"]
    #     pixel_sizes = SA[1.4u"μm", 2.2u"μm", 3.45u"μm", 5.5u"μm", 9.0u"μm"]

    #     for f in focal_lengths
    #         @test f > 0.0u"mm"
    #         @test f < 200.0u"mm"  # Reasonable range for aircraft cameras
    #     end

    #     for p in pixel_sizes
    #         @test p > 0.0u"μm"
    #         @test p < 20.0u"μm"  # Reasonable range for modern sensors
    #     end
    # end

    # @testset "Inverse Projection" begin
    #     # Test inverse projection concepts with units
    #     cam_pos = WorldPoint(-1000.0u"m", 0.0u"m", 100.0u"m")
    #     cam_rot = RotZYX(0.0, 0.0, 0.0)

    #     # Project a point
    #     world_pt = WorldPoint(0.0u"m", 10.0u"m", 0.0u"m")
    #     proj_pt = project(cam_pos, cam_rot, world_pt)

    #     # The projection should be deterministic
    #     proj_pt2 = project(cam_pos, cam_rot, world_pt)
    #     @test proj_pt.x ≈ proj_pt2.x
    #     @test proj_pt.y ≈ proj_pt2.y

    #     # Test pixel to ray direction concepts
    #     pixel_point_offset = ProjectionPoint{typeof(1.0pixel), :offset}(2048.0 * pixel, 1500.0 * pixel)
    #     pixel_point_centered = ProjectionPoint{typeof(1.0pixel), :centered}(0.0 * pixel, 0.0 * pixel)

    #     ray_dir_offset = pixel_to_ray_direction(pixel_point_offset, CAMERA_CONFIG_OFFSET)
    #     ray_dir_centered = pixel_to_ray_direction(pixel_point_centered, CAMERA_CONFIG_CENTERED)

    #     @test isa(ray_dir_offset, CameraPoint)
    #     @test isa(ray_dir_centered, CameraPoint)

    #     focal_length = 25.0u"mm"
    #     pixel_size = 3.45u"μm" / pixel

    #     # Test f-number calculations
    #     aperture_diameter = 5.0u"mm"
    #     f_number = focal_length / aperture_diameter  # f/5 for 25mm lens
    #     @test f_number ≈ 5.0 rtol = 1.0e-3
    # end
end
