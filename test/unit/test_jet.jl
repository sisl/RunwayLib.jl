using Test
using RunwayLib
using JET
using Rotations
using Unitful, Unitful.DefaultSymbols
using StaticArrays

@testset "JET Type Stability Tests" begin
    @testset "Projection Functions" begin
        # Test data
        cam_pos = WorldPoint(-500.0m, 10.0m, 100.0m)
        cam_rot = RotZYX(roll = 0.02, pitch = 0.1, yaw = -0.01)
        world_pt = WorldPoint(0.0m, 25.0m, 0.0m)
        
        @testset "project - basic functionality" begin
            # Test with default camera config
            @test_opt project(cam_pos, cam_rot, world_pt)
            
            # Test with explicit camera configs
            @test_opt project(cam_pos, cam_rot, world_pt, CAMERA_CONFIG_CENTERED)
            @test_opt project(cam_pos, cam_rot, world_pt, CAMERA_CONFIG_OFFSET)
        end
    end
end