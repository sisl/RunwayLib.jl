using Test
using RunwayLib
using StaticArrays
using Rotations
using LinearAlgebra
using Unitful

@testset "Coordinate Systems" begin
        @testset "WorldPoint" begin
                # Test construction with units
                wp = WorldPoint(1.0u"m", 2.0u"m", 3.0u"m")
                @test wp.x == 1.0u"m"
                @test wp.y == 2.0u"m"
                @test wp.z == 3.0u"m"

                # Test FieldVector interface
                @test length(wp) == 3
                @test wp[1] == 1.0u"m"
                @test wp[2] == 2.0u"m"
                @test wp[3] == 3.0u"m"

                # Test arithmetic operations
                wp2 = WorldPoint(4.0u"m", 5.0u"m", 6.0u"m")
                wp_sum = wp + wp2
                @test wp_sum == WorldPoint(5.0u"m", 7.0u"m", 9.0u"m")

                # Test scalar multiplication
                wp_scaled = 2.0 * wp
                @test wp_scaled == WorldPoint(2.0u"m", 4.0u"m", 6.0u"m")

                # Test unit conversions
                wp_km = WorldPoint(0.001u"km", 0.002u"km", 0.003u"km")
                @test uconvert(u"m", wp_km.x) == 1.0u"m"
                @test uconvert(u"m", wp_km.y) == 2.0u"m"
                @test uconvert(u"m", wp_km.z) == 3.0u"m"
        end

        @testset "CameraPoint" begin
                # Test construction with units
                cp = CameraPoint(1.0u"m", 2.0u"m", 3.0u"m")
                @test cp.x == 1.0u"m"
                @test cp.y == 2.0u"m"
                @test cp.z == 3.0u"m"

                # Test FieldVector interface
                @test length(cp) == 3
                @test cp[1] == 1.0u"m"

                # Test arithmetic operations
                cp2 = CameraPoint(4.0u"m", 5.0u"m", 6.0u"m")
                cp_sum = cp + cp2
                @test cp_sum == CameraPoint(5.0u"m", 7.0u"m", 9.0u"m")

                # Test mixed units
                cp_mm = CameraPoint(1000.0u"mm", 2000.0u"mm", 3000.0u"mm")
                @test uconvert(u"m", cp_mm.x) == 1.0u"m"
                @test uconvert(u"m", cp_mm.y) == 2.0u"m"
                @test uconvert(u"m", cp_mm.z) == 3.0u"m"
        end

        @testset "ProjectionPoint" begin
                # Test construction with pixel units (default offset coordinates)
                pp = ProjectionPoint(100.0 * 1pixel, 200.0 * 1pixel)
                @test pp.x == 100.0 * 1pixel
                @test pp.y == 200.0 * 1pixel

                # Test FieldVector interface
                @test length(pp) == 2
                @test pp[1] == 100.0 * 1pixel
                @test pp[2] == 200.0 * 1pixel

                # Test arithmetic operations
                pp2 = ProjectionPoint(50.0 * 1pixel, 75.0 * 1pixel)
                pp_sum = pp + pp2
                @test pp_sum.x == 150.0 * 1pixel
                @test pp_sum.y == 275.0 * 1pixel

                # Test fractional pixels
                pp_sub = ProjectionPoint(100.5 * 1pixel, 200.25 * 1pixel)
                @test pp_sub.x == 100.5 * 1pixel
                @test pp_sub.y == 200.25 * 1pixel

                # Test centered coordinates
                pp_centered = ProjectionPoint{Float64, :centered}(-50.0 * 1pixel, 25.0 * 1pixel)
                @test pp_centered.x == -50.0 * 1pixel
                @test pp_centered.y == 25.0 * 1pixel

                # Test offset coordinates explicitly
                pp_offset = ProjectionPoint{Float64, :offset}(1024.0 * 1pixel, 768.0 * 1pixel)
                @test pp_offset.x == 1024.0 * 1pixel
                @test pp_offset.y == 768.0 * 1pixel
        end

        @testset "Coordinate Transformations" begin
                # Test world to camera transformation with units
                cam_pos = WorldPoint(0.0u"m", 0.0u"m", 0.0u"m")
                cam_rot = RotZYX(0.0, 0.0, 0.0)  # Identity rotation
                world_pt = WorldPoint(1.0u"m", 2.0u"m", 3.0u"m")

                # With identity transformation, camera point should equal world point
                cam_pt = world_pt_to_cam_pt(cam_pos, cam_rot, world_pt)
                @test cam_pt ≈ CameraPoint(1.0u"m", 2.0u"m", 3.0u"m")

                # Test inverse transformation
                world_pt_back = cam_pt_to_world_pt(cam_pos, cam_rot, cam_pt)
                @test world_pt_back ≈ world_pt

                # Test with non-trivial transformation
                cam_pos = WorldPoint(10.0u"m", 20.0u"m", 30.0u"m")
                cam_rot = RotZYX(roll=0.0, pitch=0.0, yaw=π / 2)  # 90 degree yaw rotation
                world_pt = WorldPoint(11.0u"m", 20.0u"m", 30.0u"m")  # 1 unit in x from camera

                cam_pt = world_pt_to_cam_pt(cam_pos, cam_rot, world_pt)
                # After 90 degree yaw rotation, x becomes -y in camera frame
                @test cam_pt.y ≈ -1.0u"m" atol = 1e-10u"m"
                @test abs(cam_pt.x) < 1e-10u"m"
                @test abs(cam_pt.z) < 1e-10u"m"

                # Test typical camera parameters with units
                focal_length = 25.0u"mm"
                pixel_size = 3.45u"μm"
                @test focal_length > 0.0u"mm"
                @test pixel_size > 0.0u"μm"
                @test uconvert(u"m", focal_length) ≈ 0.025u"m"
                @test uconvert(u"m", pixel_size) ≈ 3.45e-6u"m"
        end

        @testset "Camera Projection" begin
                # Test basic projection with units
                cam_pos = WorldPoint(0.0u"m", 0.0u"m", 0.0u"m")
                cam_rot = RotZYX(0.0, 0.0, 0.0)
                world_pt = WorldPoint(1.0u"m", 0.1u"m", 0.1u"m")  # Point in front of camera

                # Test with default (offset) coordinates
                proj_pt = project(cam_pos, cam_rot, world_pt)
                @test isa(proj_pt, ProjectionPoint)

                # Test with explicit coordinate systems
                proj_pt_offset = project(cam_pos, cam_rot, world_pt, CAMERA_CONFIG_OFFSET)
                proj_pt_centered = project(cam_pos, cam_rot, world_pt, CAMERA_CONFIG_CENTERED)
                
                @test isa(proj_pt_offset, ProjectionPoint)
                @test isa(proj_pt_centered, ProjectionPoint)

                # Test that points at camera position cannot be projected (division by zero)
                world_pt_at_camera = WorldPoint(0.0u"m", 0.0u"m", 0.0u"m")
                @test_throws DivideError project(cam_pos, cam_rot, world_pt_at_camera, CAMERA_CONFIG_OFFSET)
                @test_throws DivideError project(cam_pos, cam_rot, world_pt_at_camera, CAMERA_CONFIG_CENTERED)

                # Test projection consistency - points further away should project closer to center
                world_pt_near = WorldPoint(1.0u"m", 0.1u"m", 0.0u"m")
                world_pt_far = WorldPoint(10.0u"m", 1.0u"m", 0.0u"m")  # Same angle but further

                proj_near = project(cam_pos, cam_rot, world_pt_near, CAMERA_CONFIG_OFFSET)
                proj_far = project(cam_pos, cam_rot, world_pt_far, CAMERA_CONFIG_OFFSET)

                # Both should have same projection (similar triangles)
                @test proj_near.x ≈ proj_far.x atol = 1e-6 * 1pixel
                @test proj_near.y ≈ proj_far.y atol = 1e-6 * 1pixel
        end
end
