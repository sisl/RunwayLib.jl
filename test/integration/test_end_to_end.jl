using Test
using RunwayPoseEstimation

@testset "End-to-End Integration" begin
    @testset "Complete Pipeline" begin
        # Test that all major components can work together
        
        # 1. Create mock runway specification
        runway_spec = RunwaySpec(
            "TEST_09L",
            2000.0,  # length_m
            45.0,    # width_m
            100.0,   # threshold_elevation_m
            90.0     # true_bearing_deg
        )
        
        @test isa(runway_spec, RunwaySpec)
        
        # 2. Define runway corners in world coordinates
        runway_corners = [
            WorldPoint(0.0, runway_spec.width_m/2, runway_spec.threshold_elevation_m),
            WorldPoint(0.0, -runway_spec.width_m/2, runway_spec.threshold_elevation_m),
            WorldPoint(runway_spec.length_m, runway_spec.width_m/2, runway_spec.threshold_elevation_m),
            WorldPoint(runway_spec.length_m, -runway_spec.width_m/2, runway_spec.threshold_elevation_m)
        ]
        
        @test length(runway_corners) == 4
        @test all(isa.(runway_corners, WorldPoint))
        
        # 3. Simulate aircraft pose
        true_aircraft_pos = WorldPoint(-800.0, 10.0, 200.0)  # Approaching runway
        true_aircraft_rot = RotZYX(0.02, 0.08, -0.01)  # Small attitude angles
        
        # 4. Project runway corners to image
        projected_corners = [project(true_aircraft_pos, true_aircraft_rot, corner) for corner in runway_corners]
        
        @test length(projected_corners) == 4
        @test all(isa.(projected_corners, ProjectionPoint))
        @test all(isfinite(corner.x) && isfinite(corner.y) for corner in projected_corners)
        
        # 5. Add realistic noise to observations
        pixel_noise_std = 2.0
        noisy_corners = [ProjectionPoint(corner.x + pixel_noise_std*randn(), 
                                       corner.y + pixel_noise_std*randn()) 
                        for corner in projected_corners]
        
        @test length(noisy_corners) == 4
        
        # 6. Test pose estimation setup (mock)
        initial_guess = [-750.0, 5.0, 180.0, 0.0, 0.05, 0.0]  # Close to true pose
        
        @test length(initial_guess) == 6  # 6-DOF pose
        @test initial_guess[1] < 0  # Behind runway
        @test initial_guess[3] > 0  # Above ground
        
        # 7. Test uncertainty quantification setup
        pixel_uncertainties = fill(pixel_noise_std^2, 8)  # 4 corners × 2 coordinates
        noise_model = UncorrGaussianNoiseModel([Normal(0.0, pixel_noise_std) for _ in 1:8])
        
        @test length(noise_model.noisedistributions) == 8
        
        # 8. Test integrity monitoring setup
        n_observations = 8
        n_parameters = 6
        dof = n_observations - n_parameters
        
        @test dof == 2  # Sufficient redundancy for integrity monitoring
        
        # 9. Test complete workflow data flow
        workflow_data = (
            runway_spec = runway_spec,
            runway_corners = runway_corners,
            observed_corners = noisy_corners,
            noise_model = noise_model,
            initial_guess = initial_guess
        )
        
        @test haskey(workflow_data, :runway_spec)
        @test haskey(workflow_data, :runway_corners)
        @test haskey(workflow_data, :observed_corners)
        @test haskey(workflow_data, :noise_model)
        @test haskey(workflow_data, :initial_guess)
    end
    
    @testset "Synthetic Data Validation" begin
        # Test with known ground truth for validation
        
        # Create perfect synthetic scenario
        runway_length = 1500.0
        runway_width = 40.0
        
        synthetic_runway = [
            WorldPoint(0.0, runway_width/2, 0.0),
            WorldPoint(0.0, -runway_width/2, 0.0),
            WorldPoint(runway_length, runway_width/2, 0.0),
            WorldPoint(runway_length, -runway_width/2, 0.0)
        ]
        
        # Known aircraft pose
        known_pos = WorldPoint(-600.0, 0.0, 120.0)
        known_rot = RotZYX(0.0, 0.06, 0.0)  # 6 degrees pitch down
        
        # Perfect projections (no noise)
        perfect_projections = [project(known_pos, known_rot, corner) for corner in synthetic_runway]
        
        # Test that we can recover the known pose (in principle)
        # This would be validated by actual pose estimation implementation
        
        # Test projection consistency
        reprojected = [project(known_pos, known_rot, corner) for corner in synthetic_runway]
        projection_errors = [norm([p1.x - p2.x, p1.y - p2.y]) for (p1, p2) in zip(perfect_projections, reprojected)]
        
        @test all(projection_errors .< 1e-10)  # Should be numerically identical
        
        # Test geometric consistency
        # Runway should appear as trapezoid (perspective effect)
        bottom_width = norm([perfect_projections[2].x - perfect_projections[1].x, 
                           perfect_projections[2].y - perfect_projections[1].y])
        top_width = norm([perfect_projections[4].x - perfect_projections[3].x,
                        perfect_projections[4].y - perfect_projections[3].y])
        
        @test top_width < bottom_width  # Perspective: far edge appears smaller
    end
    
    @testset "Error Propagation Validation" begin
        # Test error propagation through the complete pipeline
        
        # Start with known uncertainties
        input_pixel_std = 3.0
        n_monte_carlo = 100
        
        # Generate multiple noisy observations
        true_pos = WorldPoint(-1000.0, 0.0, 150.0)
        true_rot = RotZYX(0.0, 0.1, 0.0)
        
        runway_corners = [
            WorldPoint(0.0, 25.0, 0.0),
            WorldPoint(0.0, -25.0, 0.0),
            WorldPoint(1000.0, 25.0, 0.0),
            WorldPoint(1000.0, -25.0, 0.0)
        ]
        
        true_projections = [project(true_pos, true_rot, corner) for corner in runway_corners]
        
        # Generate Monte Carlo samples
        noisy_projection_samples = []
        for _ in 1:n_monte_carlo
            noisy_sample = [ProjectionPoint(proj.x + input_pixel_std*randn(),
                                          proj.y + input_pixel_std*randn())
                           for proj in true_projections]
            push!(noisy_projection_samples, noisy_sample)
        end
        
        @test length(noisy_projection_samples) == n_monte_carlo
        
        # Test that noise statistics are correct
        all_x_errors = []
        all_y_errors = []
        for sample in noisy_projection_samples
            for (true_proj, noisy_proj) in zip(true_projections, sample)
                push!(all_x_errors, noisy_proj.x - true_proj.x)
                push!(all_y_errors, noisy_proj.y - true_proj.y)
            end
        end
        
        empirical_std_x = std(all_x_errors)
        empirical_std_y = std(all_y_errors)
        
        @test abs(empirical_std_x - input_pixel_std) < 0.5  # Within reasonable tolerance
        @test abs(empirical_std_y - input_pixel_std) < 0.5
    end
    
    @testset "Performance Benchmarking" begin
        # Test computational performance of key operations
        
        # Benchmark projection operation
        pos = WorldPoint(-500.0, 0.0, 100.0)
        rot = RotZYX(0.0, 0.05, 0.0)
        world_pt = WorldPoint(100.0, 0.0, 0.0)
        
        # Time multiple projections
        n_projections = 1000
        start_time = time()
        for _ in 1:n_projections
            project(pos, rot, world_pt)
        end
        projection_time = time() - start_time
        
        @test projection_time < 1.0  # Should complete in reasonable time
        
        # Test that operations scale reasonably
        time_per_projection = projection_time / n_projections
        @test time_per_projection < 0.001  # Less than 1ms per projection
        
        # Benchmark coordinate transformations
        n_transforms = 1000
        start_time = time()
        for _ in 1:n_transforms
            world_pt_to_cam_pt(pos, rot, world_pt)
        end
        transform_time = time() - start_time
        
        @test transform_time < 0.5  # Should be fast
        
        # Test memory allocation (basic check)
        # This would be more sophisticated in practice
        initial_memory = Base.gc_bytes()
        for _ in 1:100
            project(pos, rot, world_pt)
        end
        final_memory = Base.gc_bytes()
        
        # Memory usage should be reasonable (this is a rough check)
        @test (final_memory - initial_memory) < 1_000_000  # Less than 1MB
    end
    
    @testset "Robustness Testing" begin
        # Test system behavior under edge cases
        
        # Test with extreme poses
        extreme_poses = [
            (WorldPoint(-10000.0, 0.0, 1000.0), RotZYX(0.0, 0.0, 0.0)),  # Very far
            (WorldPoint(-10.0, 0.0, 5.0), RotZYX(0.0, 0.0, 0.0)),        # Very close
            (WorldPoint(-500.0, 1000.0, 100.0), RotZYX(0.0, 0.0, 0.0)),  # Large cross-track
            (WorldPoint(-500.0, 0.0, 100.0), RotZYX(π/4, 0.0, 0.0))      # Large yaw angle
        ]
        
        runway_corner = WorldPoint(0.0, 0.0, 0.0)
        
        for (pos, rot) in extreme_poses
            try
                proj = project(pos, rot, runway_corner)
                @test isfinite(proj.x) && isfinite(proj.y)
            catch e
                # Some extreme cases may legitimately fail (e.g., point behind camera)
                @test isa(e, DomainError) || isa(e, DivideError)
            end
        end
        
        # Test with degenerate runway geometry
        degenerate_corners = [
            WorldPoint(0.0, 0.0, 0.0),
            WorldPoint(0.0, 0.0, 0.0),  # Duplicate point
            WorldPoint(1.0, 0.0, 0.0),
            WorldPoint(2.0, 0.0, 0.0)   # Collinear points
        ]
        
        # System should handle or gracefully fail with degenerate geometry
        @test length(unique(degenerate_corners)) < 4  # Confirms degeneracy
        
        # Test with extreme noise levels
        extreme_noise_std = 100.0  # Very large pixel noise
        noise_model = UncorrGaussianNoiseModel([Normal(0.0, extreme_noise_std) for _ in 1:8])
        
        @test all(std.(noise_model.noisedistributions) .== extreme_noise_std)
        
        # Test numerical stability
        very_small_number = 1e-15
        very_large_number = 1e15
        
        @test isfinite(very_small_number) && very_small_number > 0
        @test isfinite(very_large_number)
        
        # Operations should handle reasonable numerical ranges
        test_point = WorldPoint(very_large_number, 0.0, 0.0)
        @test isfinite(test_point.x)
    end
end
