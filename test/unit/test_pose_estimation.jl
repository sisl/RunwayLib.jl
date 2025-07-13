using Test
using RunwayLib
using Rotations
using LinearAlgebra
using Unitful
using Distributions
using StaticArrays

@testset "Pose Estimation" begin
    @testset "PoseEstimate Structure" begin
        # Test pose estimate construction with units
        position = WorldPoint(1000.0u"m", 50.0u"m", 100.0u"m")
        orientation = RotZYX(roll = 0.1, pitch = 0.05, yaw = 0.02)
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
        true_cam_rot = RotZYX(roll = 0.0, pitch = 0.1, yaw = 0.0)  # Slight pitch down (≈5.7°)

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
        known_cam_rot = RotZYX(roll = 0.0, pitch = 0.1, yaw = 0.05)  # Known orientation

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

    @testset "6-DOF Pose Estimation with Random Poses" begin
        # Define standard runway corners
        runway_corners = SA[
            WorldPoint(0.0u"m", 25.0u"m", 0.0u"m"),      # near left
            WorldPoint(0.0u"m", -25.0u"m", 0.0u"m"),     # near right
            WorldPoint(3000.0u"m", 25.0u"m", 0.0u"m"),   # far left
            WorldPoint(3000.0u"m", -25.0u"m", 0.0u"m"),   # far right
        ]

        # Test multiple random poses
        for i in 1:5
            # Generate random true pose within reasonable bounds
            true_x = -3500.0 + 1500.0 * rand()  # -2000 to -500 meters
            true_y = -100.0 + 200.0 * rand()    # -100 to +100 meters
            true_z = 50.0 + 400.0 * rand()      # 50 to 450 meters
            true_roll = -0.3 + 0.6 * rand()     # ±0.3 radians (~±17°)
            true_pitch = -0.2 + 0.4 * rand()    # ±0.2 radians (~±11°)
            true_yaw = -0.3 + 0.6 * rand()      # ±0.3 radians (~±17°)

            true_pos = WorldPoint(true_x * u"m", true_y * u"m", true_z * u"m")
            true_rot = RotZYX(roll = true_roll, pitch = true_pitch, yaw = true_yaw)

            # Project runway corners to get true observations
            true_projections = [
                project(true_pos, true_rot, corner, CAMERA_CONFIG_CENTERED)
                    for corner in runway_corners
            ]

            # Add noise to observations (±2 pixels standard deviation)
            noisy_observations = [
                ProjectionPoint(
                        :centered,
                        proj.x + randn() * 0.1pixel,
                        proj.y + randn() * 0.1pixel
                    ) for proj in true_projections
            ]

            # Create noise model matching the added noise
            noise_dists = [Normal(0.0, 2.0) for _ in 1:8]  # 2 components per corner
            noise_model = UncorrGaussianNoiseModel(noise_dists)

            # Initial guess with some error
            initial_guess_pos = [
                true_x + randn() * 100.0,    # ±100m error in position
                true_y + randn() * 50.0,     # ±50m error in position
                true_z + randn() * 50.0,     # ±50m error in position
            ] * u"m"
            initial_guess_rot = [
                true_roll + randn() * 0.1,   # ±0.1 rad error in orientation
                true_pitch + randn() * 0.1,  # ±0.1 rad error in orientation
                true_yaw + randn() * 0.1,     # ±0.1 rad error in orientation
            ] * u"rad"

            # Run pose estimation
            pose_est = estimate_pose_6dof(
                runway_corners, noisy_observations,
                CAMERA_CONFIG_CENTERED;
                noise_model, initial_guess_pos, initial_guess_rot,
            )

            # Verify convergence
            @test pose_est.converged

            # Check position accuracy (should be within ~10m given 2-pixel noise)
            pos_error_x = abs(ustrip(pose_est.position.x - true_pos.x))
            pos_error_y = abs(ustrip(pose_est.position.y - true_pos.y))
            pos_error_z = abs(ustrip(pose_est.position.z - true_pos.z))

            @test pos_error_x < 20.0
            @test pos_error_y < 20.0
            @test pos_error_z < 20.0

            # Check orientation accuracy (should be within ~0.05 radians given noise)
            rot_error_roll = abs(pose_est.orientation.theta1 - true_rot.theta1)
            rot_error_pitch = abs(pose_est.orientation.theta2 - true_rot.theta2)
            rot_error_yaw = abs(pose_est.orientation.theta3 - true_rot.theta3)

            @test rot_error_roll < 0.1
            @test rot_error_pitch < 0.1
            @test rot_error_yaw < 0.1

            # Check residual is reasonable
            @test ustrip(pose_est.residual_norm) < 10.0
        end
    end

    @testset "3-DOF Position Estimation with Random Poses" begin
        # Define standard runway corners
        runway_corners = SA[
            WorldPoint(0.0u"m", 25.0u"m", 0.0u"m"),
            WorldPoint(0.0u"m", -25.0u"m", 0.0u"m"),
            WorldPoint(3000.0u"m", 25.0u"m", 0.0u"m"),
            WorldPoint(3000.0u"m", -25.0u"m", 0.0u"m"),
        ]

        # Test multiple random poses with known orientation
        for i in 1:5
            # Generate random true position
            true_x = -3500.0 + 1000.0 * rand()  # -1500 to -500 meters
            true_y = -80.0 + 160.0 * rand()     # -80 to +80 meters
            true_z = 60.0 + 300.0 * rand()      # 60 to 360 meters

            # Fixed known orientation
            known_roll = 0.05 * randn()          # Small roll angle
            known_pitch = 0.1 + 0.05 * randn()  # Slight pitch down
            known_yaw = 0.05 * randn()           # Small yaw angle

            true_pos = WorldPoint(true_x * u"m", true_y * u"m", true_z * u"m")
            known_rot = RotZYX(roll = known_roll, pitch = known_pitch, yaw = known_yaw)

            # Project runway corners to get true observations
            true_projections = [
                project(true_pos, known_rot, corner, CAMERA_CONFIG_CENTERED)
                    for corner in runway_corners
            ]

            # Add noise to observations
            noisy_observations = [
                ProjectionPoint(
                        :centered,
                        proj.x + randn() * 0.5pixel,
                        proj.y + randn() * 0.5pixel
                    ) for proj in true_projections
            ]

            # Create noise model
            noise_dists = [Normal(0.0, 1.5) for _ in 1:8]
            noise_model = UncorrGaussianNoiseModel(noise_dists)

            # Initial position guess with some error
            initial_guess = [
                true_x + randn() * 80.0,   # ±80m error
                true_y + randn() * 30.0,   # ±30m error
                true_z + randn() * 40.0,    # ±40m error
            ] * u"m"

            # Run 3-DOF position estimation
            pose_est = estimate_pose_3dof(
                runway_corners, noisy_observations, known_rot, CAMERA_CONFIG_CENTERED;
                noise_model = noise_model,
                initial_guess_pos = initial_guess
            )

            # Verify convergence
            @test pose_est.converged

            # Check position accuracy
            pos_error_x = abs(ustrip(pose_est.position.x - true_pos.x))
            pos_error_y = abs(ustrip(pose_est.position.y - true_pos.y))
            pos_error_z = abs(ustrip(pose_est.position.z - true_pos.z))

            @test pos_error_x < 50.0
            @test pos_error_y < 15.0
            @test pos_error_z < 15.0

            # Check that orientation matches the known orientation
            @test pose_est.orientation.theta1 ≈ known_rot.theta1 atol = 1.0e-10
            @test pose_est.orientation.theta2 ≈ known_rot.theta2 atol = 1.0e-10
            @test pose_est.orientation.theta3 ≈ known_rot.theta3 atol = 1.0e-10

            # Check residual is reasonable
            @test ustrip(pose_est.residual_norm) < 8.0
        end
    end

    @testset "Pose Estimation Edge Cases" begin
        runway_corners = SA[
            WorldPoint(0.0u"m", 25.0u"m", 0.0u"m"),
            WorldPoint(0.0u"m", -25.0u"m", 0.0u"m"),
            WorldPoint(3000.0u"m", 25.0u"m", 0.0u"m"),
            WorldPoint(3000.0u"m", -25.0u"m", 0.0u"m"),
        ]

        # Test with very small noise (should converge to exact solution)
        true_pos = WorldPoint(-2800.0u"m", 20.0u"m", 220.0u"m")
        true_rot = RotZYX(roll = 0.02, pitch = 0.08, yaw = -0.03)

        true_projections = [
            project(true_pos, true_rot, corner, CAMERA_CONFIG_CENTERED)
                for corner in runway_corners
        ]

        # Add tiny amount of noise
        tiny_noise_observations = [
            ProjectionPoint(
                    :offset,
                    proj.x + randn() * 0.01pixel,
                    proj.y + randn() * 0.01pixel
                ) for proj in true_projections
        ]

        noise_model = UncorrGaussianNoiseModel([Normal(0.0, 1.0) for _ in 1:8])


        optimization_config = RunwayLib.OptimizationConfig(
            10_000,           # max_iterations
            1.0e-12,         # convergence_tolerance
            1.0e-12,          # step_tolerance
            1.0e-8           # gradient_tolerance
        )

        pose_est = estimate_pose_6dof(
            runway_corners, tiny_noise_observations, CAMERA_CONFIG_OFFSET;
            noise_model = noise_model,
            initial_guess_pos = [true_pos.x, true_pos.y, true_pos.z],
            initial_guess_rot = [true_rot.theta1, true_rot.theta2, true_rot.theta3] * u"rad",
            optimization_config
        )
        display(pose_est.position - true_pos)
        display(Rotations.params(pose_est.orientation) - Rotations.params(true_rot))

        @test pose_est.converged
        @test abs(ustrip(u"m", (pose_est.position - true_pos).x)) < 1.0  # Very accurate
        @test abs(ustrip(u"m", (pose_est.position - true_pos).y)) < 1.0
        @test abs(ustrip(u"m", (pose_est.position - true_pos).z)) < 1.0
        @test abs(Rotations.params(pose_est.orientation)[1] - Rotations.params(true_rot)[1]) < deg2rad(3.0)
        @test abs(Rotations.params(pose_est.orientation)[2] - Rotations.params(true_rot)[2]) < deg2rad(3.0)
        @test abs(Rotations.params(pose_est.orientation)[3] - Rotations.params(true_rot)[3]) < deg2rad(3.0)
        @test ustrip(pose_est.residual_norm) < 1.0  # Very small residual
    end
end
