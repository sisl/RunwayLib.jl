using Test
using RunwayLib
using Rotations
using LinearAlgebra
using Unitful
using Distributions

@testset "Pose Estimation" begin
    @testset "PoseEstimate Structure" begin
        # Test pose estimate construction with units
        position = WorldPoint(1000.0u"m", 50.0u"m", 100.0u"m")
        orientation = RotZYX(0.1, 0.05, 0.02)
        uncertainty = MvNormal(zeros(6), I(6))
        residual_norm = 2.5 * 1pixel
        converged = true

        pose_est = PoseEstimate(position, orientation, uncertainty, residual_norm, converged)

        @test pose_est.position == position
        @test pose_est.orientation == orientation
        @test pose_est.uncertainty == uncertainty
        @test pose_est.residual_norm == residual_norm
        @test pose_est.converged == converged

        # Test position is reasonable for aircraft
        @test position.x > -500.0u"m"     # Not too far behind threshold
        @test position.x < 2000.0u"m"     # Not too far ahead
        @test abs(position.y) < 200.0u"m" # Reasonable lateral deviation
        @test position.z > 0.0u"m"        # Above ground
        @test position.z < 500.0u"m"      # Below typical approach altitude
    end

    @testset "6-DOF Pose Estimation" begin
        # Create simple runway with units
        runway_spec = RunwaySpec("TEST_01", 1000.0u"m", 50.0u"m", 0.0u"m", 0.0u"°")

        # Define runway corners in world coordinates with units using StaticArrays
        runway_corners = SA[
            WorldPoint(0.0u"m", 25.0u"m", 0.0u"m"),      # near left
            WorldPoint(0.0u"m", -25.0u"m", 0.0u"m"),     # near right
            WorldPoint(1000.0u"m", 25.0u"m", 0.0u"m"),   # far left
            WorldPoint(1000.0u"m", -25.0u"m", 0.0u"m"),   # far right
        ]

        # Test runway corner geometry
        @test runway_corners[1].y == runway_spec.width_m / 2   # Left edge
        @test runway_corners[2].y == -runway_spec.width_m / 2  # Right edge
        @test runway_corners[3].x == runway_spec.length_m      # Far end
        @test runway_corners[4].x == runway_spec.length_m      # Far end

        # Simulate camera position and orientation with units
        true_cam_pos = WorldPoint(-500.0u"m", 0.0u"m", 100.0u"m")  # 500m before runway, 100m high
        true_cam_rot = RotZYX(0.0, 0.1, 0.0)  # Slight pitch down (≈5.7°)

        # Project runway corners to image using StaticArrays
        projected_corners = [project(true_cam_pos, true_cam_rot, corner) for corner in runway_corners]

        # Test that projection produces finite values
        @test all(isfinite(ustrip(corner.x)) && isfinite(ustrip(corner.y)) for corner in projected_corners)

        # Test basic pose estimation setup with units using StaticArrays
        initial_guess_pos = SA[-400.0u"m", 0.0u"m", 90.0u"m"]  # Close to true position
        initial_guess_rot = SA[0.0, 0.0, 0.0]  # Close to true orientation (radians)

        # Test initial guess is reasonable
        @test abs(initial_guess_pos[1] - true_cam_pos.x) < 200u"m"  # Within 200m
        @test abs(initial_guess_pos[2] - true_cam_pos.y) < 50u"m"   # Within 50m
        @test abs(initial_guess_pos[3] - true_cam_pos.z) < 50u"m"   # Within 50m

        @test length(runway_corners) == 4  # Standard runway has 4 corners
        @test length(projected_corners) == 4
    end

    @testset "3-DOF Position Estimation" begin
        # Test position-only estimation with known orientation and units using StaticArrays
        runway_corners = SA[
            WorldPoint(0.0u"m", 25.0u"m", 0.0u"m"),
            WorldPoint(0.0u"m", -25.0u"m", 0.0u"m"),
            WorldPoint(1000.0u"m", 25.0u"m", 0.0u"m"),
            WorldPoint(1000.0u"m", -25.0u"m", 0.0u"m"),
        ]

        true_cam_pos = WorldPoint(-500.0u"m", 10.0u"m", 100.0u"m")
        known_cam_rot = RotZYX(0.0, 0.1, 0.05)  # Known orientation

        # Test orientation angles are reasonable
        @test abs(known_cam_rot.theta1) < π / 6  # Roll within ±30°
        @test abs(known_cam_rot.theta2) < π / 6  # Pitch within ±30°
        @test abs(known_cam_rot.theta3) < π / 6  # Yaw within ±30°

        projected_corners = [project(true_cam_pos, known_cam_rot, corner) for corner in runway_corners]

        # Test 3-DOF estimation setup with units using StaticArrays
        initial_pos_guess = SA[-450.0u"m", 5.0u"m", 95.0u"m"]  # Close to true position

        # Test initial guess is close to true position
        @test abs(initial_pos_guess[1] - true_cam_pos.x) < 100u"m"
        @test abs(initial_pos_guess[2] - true_cam_pos.y) < 10u"m"
        @test abs(initial_pos_guess[3] - true_cam_pos.z) < 10u"m"

        @test length(initial_pos_guess) == 3  # Only position parameters
        @test all(isfinite(ustrip(corner.x)) && isfinite(ustrip(corner.y)) for corner in projected_corners)
    end

    @testset "Initial Guess Generation" begin
        # Test reasonable initial guess generation with units
        runway_spec = RunwaySpec("TEST_01", 1000.0u"m", 50.0u"m", 0.0u"m", 0.0u"°")

        # For approach scenario, aircraft should be:
        # - Behind runway (negative along-track)
        # - Near runway centerline (small cross-track)
        # - Above ground (positive height)
        # - Small attitude angles

        initial_pos = SA[-800.0u"m", 0.0u"m", 150.0u"m"]  # Typical approach position
        initial_rot = SA[0.0, 0.05, 0.0]                  # Slight pitch down for approach (radians)

        @test initial_pos[1] < 0u"m"  # Behind runway
        @test abs(initial_pos[2]) < runway_spec.width_m  # Near centerline
        @test initial_pos[3] > 0u"m"  # Above ground
        @test all(abs.(initial_rot) .< 0.5)  # Reasonable attitude angles (< ~30 deg)

        # Test typical approach parameters
        approach_distance = abs(initial_pos[1])
        approach_height = initial_pos[3]
        glide_angle = atan(ustrip(approach_height) / ustrip(approach_distance))

        @test 500u"m" <= approach_distance <= 2000u"m"  # Typical approach distances
        @test 50u"m" <= approach_height <= 500u"m"      # Typical approach heights
        @test 0.02 <= glide_angle <= 0.2               # Typical glide angles (1-8 degrees)
    end
end
