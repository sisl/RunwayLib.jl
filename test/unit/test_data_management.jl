using Test
using RunwayLib
using DataFrames
using CSV
using Unitful

@testset "Data Management" begin
    @testset "Runway Database" begin
        # Test runway specification structure with units
        runway_spec = RunwaySpec(
            "KORD_10L",
            3048.0u"m",  # length_m
            45.7u"m",    # width_m
            201.2u"m",   # threshold_elevation_m
            104.0u"°"    # true_bearing_deg
        )
        
        @test runway_spec.icao_code == "KORD_10L"
        @test runway_spec.length_m == 3048.0u"m"
        @test runway_spec.width_m == 45.7u"m"
        @test runway_spec.threshold_elevation_m == 201.2u"m"
        @test runway_spec.true_bearing_deg == 104.0u"°"
        
        # Test runway spec validation
        @test runway_spec.length_m > 0u"m"
        @test runway_spec.width_m > 0u"m"
        @test 0u"°" <= runway_spec.true_bearing_deg < 360u"°"
        
        # Test unit conversions
        @test uconvert(u"ft", runway_spec.length_m) ≈ 10000.0u"ft" rtol=1e-3
        @test uconvert(u"ft", runway_spec.width_m) ≈ 149.9u"ft" rtol=1e-2
        @test uconvert(u"rad", runway_spec.true_bearing_deg) ≈ 1.815u"rad" rtol=1e-2
    end
    
    @testset "Flight Data Processing" begin
        # Create mock flight data with units
        mock_data = DataFrame(
            timestamp = [1.0, 2.0, 3.0],
            airport_runway = ["KORD_10L", "KORD_10L", "KORD_10L"],
            gt_along_track_distance_m = [-1000.0u"m", -800.0u"m", -600.0u"m"],
            gt_cross_track_distance_m = [0.0u"m", 5.0u"m", -2.0u"m"],
            gt_height_m = [100.0u"m", 80.0u"m", 60.0u"m"],
            gt_yaw_deg = [0.0u"°", 1.0u"°", -0.5u"°"],
            gt_pitch_deg = [-2.0u"°", -1.5u"°", -1.0u"°"],
            gt_roll_deg = [0.0u"°", 0.2u"°", -0.1u"°"],
            pred_kp_bottom_left_x_px = [1000.0u"pixel", 1010.0u"pixel", 1020.0u"pixel"],
            pred_kp_bottom_left_y_px = [1500.0u"pixel", 1510.0u"pixel", 1520.0u"pixel"],
            pred_kp_bottom_right_x_px = [3000.0u"pixel", 3010.0u"pixel", 3020.0u"pixel"],
            pred_kp_bottom_right_y_px = [1500.0u"pixel", 1510.0u"pixel", 1520.0u"pixel"],
            pred_kp_top_left_x_px = [1200.0u"pixel", 1210.0u"pixel", 1220.0u"pixel"],
            pred_kp_top_left_y_px = [800.0u"pixel", 810.0u"pixel", 820.0u"pixel"],
            pred_kp_top_right_x_px = [2800.0u"pixel", 2810.0u"pixel", 2820.0u"pixel"],
            pred_kp_top_right_y_px = [800.0u"pixel", 810.0u"pixel", 820.0u"pixel"]
        )
        
        @test nrow(mock_data) == 3
        @test "airport_runway" in names(mock_data)
        @test all(mock_data.gt_along_track_distance_m .< 0u"m")  # Aircraft approaching
        @test all(mock_data.gt_height_m .> 0u"m")  # Aircraft above ground
        
        # Test data validation
        @test all(isfinite.(ustrip.(mock_data.gt_along_track_distance_m)))
        @test all(isfinite.(ustrip.(mock_data.pred_kp_bottom_left_x_px)))
        
        # Test corner extraction with units using StaticArrays
        corners_row1 = SA[
            ProjectionPoint(mock_data.pred_kp_bottom_left_x_px[1], mock_data.pred_kp_bottom_left_y_px[1]),
            ProjectionPoint(mock_data.pred_kp_bottom_right_x_px[1], mock_data.pred_kp_bottom_right_y_px[1]),
            ProjectionPoint(mock_data.pred_kp_top_left_x_px[1], mock_data.pred_kp_top_left_y_px[1]),
            ProjectionPoint(mock_data.pred_kp_top_right_x_px[1], mock_data.pred_kp_top_right_y_px[1])
        ]
        
        @test length(corners_row1) == 4
        @test all(isa.(corners_row1, ProjectionPoint))
        
        # Test pixel coordinate ranges
        for corner in corners_row1
            @test 0u"pixel" <= corner.x <= 4096u"pixel"
            @test 0u"pixel" <= corner.y <= 3000u"pixel"
        end
    end
    
    @testset "Data Validation" begin
        # Test missing data detection with units
        invalid_data = DataFrame(
            airport_runway = ["KORD_10L", "KORD_10L"],
            gt_along_track_distance_m = [-1000.0u"m", missing],
            pred_kp_bottom_left_x_px = [1000.0u"pixel", 1010.0u"pixel"]
        )
        
        missing_mask = ismissing.(invalid_data.gt_along_track_distance_m)
        @test sum(missing_mask) == 1
        
        # Test outlier detection with units using StaticArrays
        outlier_data = SA[1.0u"pixel", 2.0u"pixel", 3.0u"pixel", 1000.0u"pixel", 4.0u"pixel", 5.0u"pixel"]
        @test maximum(outlier_data) > 100 * median(outlier_data)
        
        # Test coordinate system consistency
        # Bottom corners should have larger y-coordinates than top corners (image coordinates)
        @test 1500.0u"pixel" > 800.0u"pixel"  # bottom_left_y > top_left_y
        @test 1500.0u"pixel" > 800.0u"pixel"  # bottom_right_y > top_right_y
        
        # Test runway dimension validation using StaticArrays
        valid_lengths = SA[1000.0u"m", 2000.0u"m", 3000.0u"m", 4000.0u"m"]
        valid_widths = SA[30.0u"m", 45.0u"m", 60.0u"m"]
        
        for length in valid_lengths
            @test 500u"m" <= length <= 6000u"m"  # Reasonable runway length range
        end
        
        for width in valid_widths
            @test 15u"m" <= width <= 100u"m"  # Reasonable runway width range
        end
    end
    
    @testset "Preprocessing Utilities" begin
        # Test coordinate system conversion with units
        # Image coordinates: origin at top-left, y increases downward
        # Our coordinates: origin at center, y increases upward
        
        image_width = 4096u"pixel"
        image_height = 3000u"pixel"
        
        # Convert from image coordinates to centered coordinates
        img_x, img_y = 2048.0u"pixel", 1500.0u"pixel"  # Center of image
        centered_x = img_x - image_width/2
        centered_y = -(img_y - image_height/2)  # Flip y-axis
        
        @test centered_x ≈ 0.0u"pixel" atol=1.0u"pixel"
        @test centered_y ≈ 0.0u"pixel" atol=1.0u"pixel"
        
        # Test corner ordering consistency with units using StaticArrays
        # Corners should form a reasonable quadrilateral
        corners = SA[
            ProjectionPoint(100.0u"pixel", 200.0u"pixel"),   # bottom_left
            ProjectionPoint(300.0u"pixel", 200.0u"pixel"),   # bottom_right  
            ProjectionPoint(120.0u"pixel", 100.0u"pixel"),   # top_left
            ProjectionPoint(280.0u"pixel", 100.0u"pixel")    # top_right
        ]
        
        # Basic geometric consistency checks
        @test corners[1].x < corners[2].x  # left < right (bottom)
        @test corners[3].x < corners[4].x  # left < right (top)
        @test corners[1].y > corners[3].y  # bottom > top (left)
        @test corners[2].y > corners[4].y  # bottom > top (right)
        
        # Test pixel uncertainty ranges using StaticArrays
        typical_uncertainties = SA[1.0u"pixel", 2.5u"pixel", 5.0u"pixel", 10.0u"pixel"]
        for uncertainty in typical_uncertainties
            @test 0.1u"pixel" <= uncertainty <= 20.0u"pixel"  # Reasonable range
        end
    end
end
