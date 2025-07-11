using Test
using RunwayPoseEstimation
using Distributions
using LinearAlgebra

@testset "Integrity Monitoring" begin
    @testset "RAIM Statistic Computation" begin
        # Test basic RAIM statistic computation setup
        
        # Mock pose estimate
        estimated_pos = WorldPoint(-500.0, 5.0, 100.0)
        estimated_rot = RotZYX(0.0, 0.1, 0.02)
        
        # Mock runway corners
        runway_corners = [
            WorldPoint(0.0, 25.0, 0.0),
            WorldPoint(0.0, -25.0, 0.0),
            WorldPoint(1000.0, 25.0, 0.0),
            WorldPoint(1000.0, -25.0, 0.0)
        ]
        
        # Mock observed corners
        observed_corners = [
            ProjectionPoint(-80.0, 40.0),
            ProjectionPoint(-80.0, -40.0),
            ProjectionPoint(80.0, 30.0),
            ProjectionPoint(80.0, -30.0)
        ]
        
        # Mock noise model
        pixel_std = 2.0
        noise_cov = Diagonal(fill(pixel_std^2, 8))  # 4 corners × 2 coordinates
        
        @test size(noise_cov) == (8, 8)
        @test all(diag(noise_cov) .> 0)
        
        # Test residual computation concept
        predicted_corners = [project(estimated_pos, estimated_rot, corner) for corner in runway_corners]
        residuals = vcat([[obs.x - pred.x, obs.y - pred.y] for (obs, pred) in zip(observed_corners, predicted_corners)]...)
        
        @test length(residuals) == 8
        @test all(isfinite.(residuals))
        
        # Test chi-squared statistic computation
        # χ² = r' Σ⁻¹ r where r is residual vector, Σ is noise covariance
        chi_squared_stat = residuals' * inv(noise_cov) * residuals
        @test chi_squared_stat >= 0
        @test isfinite(chi_squared_stat)
    end
    
    @testset "Chi-Squared Test" begin
        # Test chi-squared distribution properties for RAIM
        
        # Degrees of freedom = n_observations - n_parameters
        n_observations = 8  # 4 corners × 2 coordinates
        n_parameters = 6    # 6-DOF pose (3 position + 3 rotation)
        dof = n_observations - n_parameters
        
        @test dof == 2
        @test dof > 0  # Must have redundancy for integrity monitoring
        
        # Test chi-squared distribution
        chi_sq_dist = Chisq(dof)
        @test isa(chi_sq_dist, Chisq)
        
        # Test critical values
        significance_level = 0.05
        critical_value = quantile(chi_sq_dist, 1 - significance_level)
        @test critical_value > 0
        
        # Test p-value computation
        test_statistic = 3.0
        p_value = 1 - cdf(chi_sq_dist, test_statistic)
        @test 0 <= p_value <= 1
        
        # Test integrity decision
        integrity_ok = test_statistic <= critical_value
        @test isa(integrity_ok, Bool)
    end
    
    @testset "Fault Detection" begin
        # Test fault detection concepts
        
        # Normal case: small residuals
        normal_residuals = [0.5, -0.3, 0.8, -0.2, 0.1, 0.4, -0.6, 0.3]
        normal_chi_sq = sum(normal_residuals.^2) / 4.0  # Normalized by variance
        
        # Faulty case: one large residual (outlier)
        faulty_residuals = [0.5, -0.3, 15.0, -0.2, 0.1, 0.4, -0.6, 0.3]  # Large error in 3rd observation
        faulty_chi_sq = sum(faulty_residuals.^2) / 4.0
        
        @test faulty_chi_sq > normal_chi_sq
        @test faulty_chi_sq > 10 * normal_chi_sq  # Should be significantly larger
        
        # Test fault isolation concept
        # Remove suspected faulty observation and recompute
        suspected_fault_idx = 3
        reduced_residuals = faulty_residuals[setdiff(1:8, suspected_fault_idx)]
        reduced_chi_sq = sum(reduced_residuals.^2) / 4.0
        
        @test reduced_chi_sq < faulty_chi_sq
        @test reduced_chi_sq ≈ normal_chi_sq atol=1.0
    end
    
    @testset "Performance Metrics" begin
        # Test integrity monitoring performance metrics
        
        # Mock test results
        n_tests = 1000
        true_faults = rand(Bool, n_tests)  # Random true fault status
        detected_faults = copy(true_faults)
        
        # Add some false alarms and missed detections
        false_alarm_rate = 0.05
        missed_detection_rate = 0.10
        
        # Simulate false alarms (detect fault when none exists)
        no_fault_indices = findall(.!true_faults)
        n_false_alarms = round(Int, false_alarm_rate * length(no_fault_indices))
        false_alarm_indices = no_fault_indices[1:n_false_alarms]
        detected_faults[false_alarm_indices] .= true
        
        # Simulate missed detections (miss fault when it exists)
        fault_indices = findall(true_faults)
        n_missed = round(Int, missed_detection_rate * length(fault_indices))
        missed_indices = fault_indices[1:n_missed]
        detected_faults[missed_indices] .= false
        
        # Compute confusion matrix
        true_positives = sum(true_faults .& detected_faults)
        false_positives = sum(.!true_faults .& detected_faults)
        true_negatives = sum(.!true_faults .& .!detected_faults)
        false_negatives = sum(true_faults .& .!detected_faults)
        
        @test true_positives + false_positives + true_negatives + false_negatives == n_tests
        
        # Compute performance metrics
        computed_false_alarm_rate = false_positives / (false_positives + true_negatives)
        computed_missed_detection_rate = false_negatives / (false_negatives + true_positives)
        
        @test computed_false_alarm_rate ≈ false_alarm_rate atol=0.02
        @test computed_missed_detection_rate ≈ missed_detection_rate atol=0.02
    end
    
    @testset "Integrity Risk Assessment" begin
        # Test integrity risk computation
        
        # Mock system parameters
        false_alarm_probability = 0.05
        missed_detection_probability = 0.10
        fault_probability = 0.01  # Prior probability of fault
        
        @test 0 <= false_alarm_probability <= 1
        @test 0 <= missed_detection_probability <= 1
        @test 0 <= fault_probability <= 1
        
        # Compute integrity risk using Bayes' theorem
        # P(fault | no alarm) = P(no alarm | fault) * P(fault) / P(no alarm)
        prob_no_alarm_given_fault = missed_detection_probability
        prob_no_alarm_given_no_fault = 1 - false_alarm_probability
        prob_no_alarm = prob_no_alarm_given_fault * fault_probability + 
                       prob_no_alarm_given_no_fault * (1 - fault_probability)
        
        integrity_risk = prob_no_alarm_given_fault * fault_probability / prob_no_alarm
        
        @test 0 <= integrity_risk <= 1
        @test integrity_risk < fault_probability  # Should be reduced by monitoring
        
        # Test that better detection reduces integrity risk
        better_missed_detection_prob = 0.05  # Better than 0.10
        better_prob_no_alarm_given_fault = better_missed_detection_prob
        better_prob_no_alarm = better_prob_no_alarm_given_fault * fault_probability + 
                              prob_no_alarm_given_no_fault * (1 - fault_probability)
        better_integrity_risk = better_prob_no_alarm_given_fault * fault_probability / better_prob_no_alarm
        
        @test better_integrity_risk < integrity_risk
    end
    
    @testset "RAIM Availability" begin
        # Test RAIM availability conditions
        
        # Need sufficient redundancy for fault detection
        min_observations = 4  # 4 corners minimum
        min_parameters = 6    # 6-DOF pose
        
        # For fault detection, need at least 1 degree of freedom
        @test min_observations * 2 > min_parameters  # 8 > 6
        
        # For fault isolation, need even more redundancy
        min_dof_for_isolation = 2
        @test min_observations * 2 - min_parameters >= min_dof_for_isolation
        
        # Test geometric dilution of precision (GDOP) concept
        # Better corner geometry leads to better RAIM performance
        
        # Good geometry: corners well separated
        good_corners = [
            ProjectionPoint(-200.0, 150.0),   # Well separated
            ProjectionPoint(200.0, 150.0),
            ProjectionPoint(-180.0, -150.0),
            ProjectionPoint(180.0, -150.0)
        ]
        
        # Poor geometry: corners clustered
        poor_corners = [
            ProjectionPoint(-10.0, 10.0),     # Clustered together
            ProjectionPoint(10.0, 10.0),
            ProjectionPoint(-10.0, -10.0),
            ProjectionPoint(10.0, -10.0)
        ]
        
        # Compute corner spread as simple geometry metric
        good_spread = maximum([abs(c.x) + abs(c.y) for c in good_corners])
        poor_spread = maximum([abs(c.x) + abs(c.y) for c in poor_corners])
        
        @test good_spread > poor_spread
        @test good_spread > 5 * poor_spread  # Significantly better geometry
    end
end
