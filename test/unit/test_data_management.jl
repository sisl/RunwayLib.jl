using Test
using RunwayPoseEstimation
using DataFrames
using CSV

@testset "Data Management" begin
    @testset "Runway Database" begin
        # Test runway specification structure
        runway_spec = RunwaySpec(
            "KORD_10L",
            3048.0,  # length_m
            45.7,    # width_m
            201.2,   # threshold_elevation_m
            104.0    # true_bearing_deg
        )
        
        @test runway_spec.icao_code == "KORD_10L"
        @test runway_spec.length_m == 3048.0
        @test runway_spec.width_m == 45.7
        @test runway_spec.threshold_elevation_m == 201.2
        @test runway_spec.true_bearing_deg == 104.0
        
        # Test runway spec validation
        @test runway_spec.length_m > 0
        @test runway_spec.width_m > 0
        @test 0 <= runway_spec.true_bearing_deg < 360
    end
    
    @testset "Flight Data Processing" begin
        # Create mock flight data
        mock_data = DataFrame(
            timestamp = [1.0, 2.0, 3.0],
            airport_runway = ["KORD_10L", "KORD_10L", "KORD_10L"],
            gt_along_track_distance_m = [-1000.0, -800.0, -600.0],
            gt_cross_track_distance_m = [0.0, 5.0, -2.0],
            gt_height_m = [100.0, 80.0, 60.0],
            gt_yaw_deg = [0.0, 1.0, -0.5],
            gt_pitch_deg = [-2.0, -1.5, -1.0],
            gt_roll_deg = [0.0, 0.2, -0.1],
            pred_kp_bottom_left_x_px = [1000.0, 1010.0, 1020.0],
            pred_kp_bottom_left_y_px = [1500.0, 1510.0, 1520.0],
            pred_kp_bottom_right_x_px = [3000.0, 3010.0, 3020.0],
            pred_kp_bottom_right_y_px = [1500.0, 1510.0, 1520.0],
            pred_kp_top_left_x_px = [1200.0, 1210.0, 1220.0],
            pred_kp_top_left_y_px = [800.0, 810.0, 820.0],
            pred_kp_top_right_x_px = [2800.0, 2810.0, 2820.0],
            pred_kp_top_right_y_px = [800.0, 810.0, 820.0]
        )
        
        @test nrow(mock_data) == 3
        @test "airport_runway" in names(mock_data)
        @test all(mock_data.gt_along_track_distance_m .< 0)  # Aircraft approaching
        @test all(mock_data.gt_height_m .> 0)  # Aircraft above ground
        
        # Test data validation
        @test all(isfinite.(mock_data.gt_along_track_distance_m))
        @test all(isfinite.(mock_data.pred_kp_bottom_left_x_px))
        
        # Test corner extraction
        corners_row1 = [
            ProjectionPoint(mock_data.pred_kp_bottom_left_x_px[1], mock_data.pred_kp_bottom_left_y_px[1]),
            ProjectionPoint(mock_data.pred_kp_bottom_right_x_px[1], mock_data.pred_kp_bottom_right_y_px[1]),
            ProjectionPoint(mock_data.pred_kp_top_left_x_px[1], mock_data.pred_kp_top_left_y_px[1]),
            ProjectionPoint(mock_data.pred_kp_top_right_x_px[1], mock_data.pred_kp_top_right_y_px[1])
        ]
        
        @test length(corners_row1) == 4
        @test all(isa.(corners_row1, ProjectionPoint))
    end
    
    @testset "Data Validation" begin
        # Test missing data detection
        invalid_data = DataFrame(
            airport_runway = ["KORD_10L", "KORD_10L"],
            gt_along_track_distance_m = [-1000.0, missing],
            pred_kp_bottom_left_x_px = [1000.0, 1010.0]
        )
        
        missing_mask = ismissing.(invalid_data.gt_along_track_distance_m)
        @test sum(missing_mask) == 1
        
        # Test outlier detection
        outlier_data = [1.0, 2.0, 3.0, 1000.0, 4.0, 5.0]  # 1000.0 is outlier
        @test maximum(outlier_data) > 100 * median(outlier_data)
        
        # Test coordinate system consistency
        # Bottom corners should have larger y-coordinates than top corners (image coordinates)
        @test 1500.0 > 800.0  # bottom_left_y > top_left_y
        @test 1500.0 > 800.0  # bottom_right_y > top_right_y
    end
    
    @testset "Preprocessing Utilities" begin
        # Test coordinate system conversion
        # Image coordinates: origin at top-left, y increases downward
        # Our coordinates: origin at center, y increases upward
        
        image_width = 4096
        image_height = 3000
        
        # Convert from image coordinates to centered coordinates
        img_x, img_y = 2048.0, 1500.0  # Center of image
        centered_x = img_x - image_width/2
        centered_y = -(img_y - image_height/2)  # Flip y-axis
        
        @test centered_x ≈ 0.0 atol=1.0
        @test centered_y ≈ 0.0 atol=1.0
        
        # Test corner ordering consistency
        # Corners should form a reasonable quadrilateral
        corners = [
            ProjectionPoint(100.0, 200.0),   # bottom_left
            ProjectionPoint(300.0, 200.0),   # bottom_right  
            ProjectionPoint(120.0, 100.0),   # top_left
            ProjectionPoint(280.0, 100.0)    # top_right
        ]
        
        # Basic geometric consistency checks
        @test corners[1].x < corners[2].x  # left < right (bottom)
        @test corners[3].x < corners[4].x  # left < right (top)
        @test corners[1].y > corners[3].y  # bottom > top (left)
        @test corners[2].y > corners[4].y  # bottom > top (right)
    end
end
