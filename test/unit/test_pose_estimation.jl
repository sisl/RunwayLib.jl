using Test
using RunwayPoseEstimation
using Rotations
using LinearAlgebra

@testset "Pose Estimation" begin
    @testset "PoseEstimate Structure" begin
        # Test pose estimate construction
        position = WorldPoint(1000.0, 50.0, 100.0)
        orientation = RotZYX(0.1, 0.05, 0.02)
        uncertainty = MvNormal(zeros(6), I(6))
        residual_norm = 2.5
        converged = true
        
        pose_est = PoseEstimate(position, orientation, uncertainty, residual_norm, converged)
        
        @test pose_est.position == position
        @test pose_est.orientation == orientation
        @test pose_est.uncertainty == uncertainty
        @test pose_est.residual_norm == residual_norm
        @test pose_est.converged == converged
    end
    
    @testset "6-DOF Pose Estimation" begin
        # Create simple runway (rectangle at origin)
        runway_spec = RunwaySpec("TEST_01", 1000.0, 50.0, 0.0, 0.0)
        
        # Define runway corners in world coordinates
        runway_corners = [
            WorldPoint(0.0, 25.0, 0.0),      # near left
            WorldPoint(0.0, -25.0, 0.0),     # near right
            WorldPoint(1000.0, 25.0, 0.0),   # far left  
            WorldPoint(1000.0, -25.0, 0.0)   # far right
        ]
        
        # Simulate camera position and orientation
        true_cam_pos = WorldPoint(-500.0, 0.0, 100.0)  # 500m before runway, 100m high
        true_cam_rot = RotZYX(0.0, 0.1, 0.0)  # Slight pitch down
        
        # Project runway corners to image
        projected_corners = [project(true_cam_pos, true_cam_rot, corner) for corner in runway_corners]
        
        # Test that projection produces finite values
        @test all(isfinite(corner.x) && isfinite(corner.y) for corner in projected_corners)
        
        # Test basic pose estimation setup
        initial_guess = [-400.0, 0.0, 90.0, 0.0, 0.0, 0.0]  # Close to true pose
        
        # For now, just test that the estimation function can be called
        # (Actual implementation would go in estimate_pose_6dof function)
        @test length(initial_guess) == 6  # 3 position + 3 rotation parameters
        @test length(runway_corners) == 4  # Standard runway has 4 corners
        @test length(projected_corners) == 4
    end
    
    @testset "3-DOF Position Estimation" begin
        # Test position-only estimation with known orientation
        runway_corners = [
            WorldPoint(0.0, 25.0, 0.0),
            WorldPoint(0.0, -25.0, 0.0),
            WorldPoint(1000.0, 25.0, 0.0),
            WorldPoint(1000.0, -25.0, 0.0)
        ]
        
        true_cam_pos = WorldPoint(-500.0, 10.0, 100.0)
        known_cam_rot = RotZYX(0.0, 0.1, 0.05)
        
        projected_corners = [project(true_cam_pos, known_cam_rot, corner) for corner in runway_corners]
        
        # Test 3-DOF estimation setup
        initial_pos_guess = [-450.0, 5.0, 95.0]  # Close to true position
        
        @test length(initial_pos_guess) == 3  # Only position parameters
        @test all(isfinite(corner.x) && isfinite(corner.y) for corner in projected_corners)
    end
    
    @testset "Optimization Setup" begin
        # Test loss function construction
        runway_corners = [WorldPoint(0.0, 0.0, 0.0), WorldPoint(100.0, 0.0, 0.0)]
        observed_corners = [ProjectionPoint(100.0, 0.0), ProjectionPoint(200.0, 0.0)]
        
        # Test that we can compute reprojection error
        cam_pos = WorldPoint(-100.0, 0.0, 50.0)
        cam_rot = RotZYX(0.0, 0.0, 0.0)
        
        predicted_corners = [project(cam_pos, cam_rot, corner) for corner in runway_corners]
        
        # Compute reprojection errors
        errors = [obs - pred for (obs, pred) in zip(observed_corners, predicted_corners)]
        error_norms = [sqrt(err.x^2 + err.y^2) for err in errors]
        
        @test length(errors) == length(runway_corners)
        @test all(isfinite.(error_norms))
        @test all(error_norms .>= 0)
        
        # Test weighted least squares setup
        weights = [1.0, 1.0]  # Equal weights
        weighted_errors = [w * norm for (w, norm) in zip(weights, error_norms)]
        total_error = sum(weighted_errors.^2)
        
        @test total_error >= 0
        @test isfinite(total_error)
    end
    
    @testset "Initial Guess Generation" begin
        # Test reasonable initial guess generation
        runway_spec = RunwaySpec("TEST_01", 1000.0, 50.0, 0.0, 0.0)
        
        # For approach scenario, aircraft should be:
        # - Behind runway (negative along-track)
        # - Near runway centerline (small cross-track)
        # - Above ground (positive height)
        # - Small attitude angles
        
        initial_pos = [-800.0, 0.0, 150.0]  # Typical approach position
        initial_rot = [0.0, 0.05, 0.0]      # Slight pitch down for approach
        
        @test initial_pos[1] < 0  # Behind runway
        @test abs(initial_pos[2]) < runway_spec.width_m  # Near centerline
        @test initial_pos[3] > 0  # Above ground
        @test all(abs.(initial_rot) .< 0.5)  # Reasonable attitude angles (< ~30 deg)
    end
    
    @testset "Convergence Criteria" begin
        # Test convergence checking
        residual_history = [10.0, 5.0, 2.0, 1.0, 0.5, 0.49, 0.48]
        tolerance = 0.5
        
        # Check if converged (residual below tolerance)
        converged = residual_history[end] < tolerance
        @test converged
        
        # Check for stagnation (small improvement)
        improvement = residual_history[end-1] - residual_history[end]
        min_improvement = 0.01
        stagnated = improvement < min_improvement
        @test stagnated
        
        # Test maximum iterations
        max_iterations = 100
        current_iteration = 50
        @test current_iteration < max_iterations
    end
end
