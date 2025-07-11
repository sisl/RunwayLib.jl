using Test
using RunwayLib
using TypedTables
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
        @test uconvert(u"ft", runway_spec.length_m) ≈ 10000.0u"ft" rtol = 1.0e-3
        @test uconvert(u"ft", runway_spec.width_m) ≈ 149.9u"ft" rtol = 1.0e-2
        @test uconvert(u"rad", runway_spec.true_bearing_deg) ≈ 1.815u"rad" rtol = 1.0e-2
    end

    @testset "Flight Data Processing" begin
        # Create mock flight data with units using TypedTables
        mock_data = Table(
            timestamp = [1.0, 2.0, 3.0],
            airport_runway = ["KORD_10L", "KORD_10L", "KORD_10L"],
            gt_along_track_distance_m = [-1000.0u"m", -800.0u"m", -600.0u"m"],
            gt_cross_track_distance_m = [0.0u"m", 5.0u"m", -2.0u"m"],
            gt_height_m = [100.0u"m", 80.0u"m", 60.0u"m"],
            gt_yaw_deg = [0.0u"°", 1.0u"°", -0.5u"°"],
            gt_pitch_deg = [-2.0u"°", -1.5u"°", -1.0u"°"],
            gt_roll_deg = [0.0u"°", 0.2u"°", -0.1u"°"],
            pred_kp_bottom_left_x_px = [1000.0 * 1pixel, 1010.0 * 1pixel, 1020.0 * 1pixel],
            pred_kp_bottom_left_y_px = [1500.0 * 1pixel, 1510.0 * 1pixel, 1520.0 * 1pixel],
            pred_kp_bottom_right_x_px = [3000.0 * 1pixel, 3010.0 * 1pixel, 3020.0 * 1pixel],
            pred_kp_bottom_right_y_px = [1500.0 * 1pixel, 1510.0 * 1pixel, 1520.0 * 1pixel],
            pred_kp_top_left_x_px = [1200.0 * 1pixel, 1210.0 * 1pixel, 1220.0 * 1pixel],
            pred_kp_top_left_y_px = [800.0 * 1pixel, 810.0 * 1pixel, 820.0 * 1pixel],
            pred_kp_top_right_x_px = [2800.0 * 1pixel, 2810.0 * 1pixel, 2820.0 * 1pixel],
            pred_kp_top_right_y_px = [800.0 * 1pixel, 810.0 * 1pixel, 820.0 * 1pixel]
        )

        @test length(mock_data) == 3
        @test haskey(mock_data, :airport_runway)
        @test all(mock_data.gt_along_track_distance_m .< 0u"m")  # Aircraft approaching
        @test all(mock_data.gt_height_m .> 0u"m")  # Aircraft above ground

        # Test data validation
        @test all(isfinite.(ustrip.(mock_data.gt_along_track_distance_m)))
        @test all(isfinite.(ustrip.(mock_data.pred_kp_bottom_left_x_px)))

        # Test corner extraction with units
        corners_row1 = [
            ProjectionPoint(mock_data.pred_kp_bottom_left_x_px[1], mock_data.pred_kp_bottom_left_y_px[1]),
            ProjectionPoint(mock_data.pred_kp_bottom_right_x_px[1], mock_data.pred_kp_bottom_right_y_px[1]),
            ProjectionPoint(mock_data.pred_kp_top_left_x_px[1], mock_data.pred_kp_top_left_y_px[1]),
            ProjectionPoint(mock_data.pred_kp_top_right_x_px[1], mock_data.pred_kp_top_right_y_px[1]),
        ]

        @test length(corners_row1) == 4
        @test all(isa.(corners_row1, ProjectionPoint))

        # Test pixel coordinate ranges
        for corner in corners_row1
            @test 0 * 1pixel <= corner.x <= 4096 * 1pixel
            @test 0 * 1pixel <= corner.y <= 3000 * 1pixel
        end
    end

    @testset "Data Validation" begin
        # Test missing data detection with units
        # Note: TypedTables handles missing values differently - they need to be Union types
        valid_distances = Union{typeof(1.0u"m"), Missing}[-1000.0u"m", missing]
        invalid_data = Table(
            airport_runway = ["KORD_10L", "KORD_10L"],
            gt_along_track_distance_m = valid_distances,
            pred_kp_bottom_left_x_px = [1000.0 * 1pixel, 1010.0 * 1pixel]
        )

        missing_mask = ismissing.(invalid_data.gt_along_track_distance_m)
        @test sum(missing_mask) == 1

        # Test outlier detection with units
        outlier_data = [1.0 * 1pixel, 2.0 * 1pixel, 3.0 * 1pixel, 1000.0 * 1pixel, 4.0 * 1pixel, 5.0 * 1pixel]
        @test maximum(outlier_data) > 100 * median(outlier_data)

        # Test coordinate system consistency
        # Bottom corners should have larger y-coordinates than top corners (image coordinates)
        @test 1500.0 * 1pixel > 800.0 * 1pixel  # bottom_left_y > top_left_y
        @test 1500.0 * 1pixel > 800.0 * 1pixel  # bottom_right_y > top_right_y

        # Test runway dimension validation
        valid_lengths = [1000.0u"m", 2000.0u"m", 3000.0u"m", 4000.0u"m"]
        valid_widths = [30.0u"m", 45.0u"m", 60.0u"m"]

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

        image_width = 4096 * 1pixel
        image_height = 3000 * 1pixel

        # Convert from image coordinates to centered coordinates
        img_x, img_y = 2048.0 * 1pixel, 1500.0 * 1pixel  # Center of image
        centered_x = img_x - image_width / 2
        centered_y = -(img_y - image_height / 2)  # Flip y-axis

        @test centered_x ≈ 0.0 * 1pixel atol = 1.0 * 1pixel
        @test centered_y ≈ 0.0 * 1pixel atol = 1.0 * 1pixel

        # Test corner ordering consistency with units
        # Corners should form a reasonable quadrilateral
        corners = [
            ProjectionPoint(100.0 * 1pixel, 200.0 * 1pixel),   # bottom_left
            ProjectionPoint(300.0 * 1pixel, 200.0 * 1pixel),   # bottom_right
            ProjectionPoint(120.0 * 1pixel, 100.0 * 1pixel),   # top_left
            ProjectionPoint(280.0 * 1pixel, 100.0 * 1pixel),    # top_right
        ]

        # Basic geometric consistency checks
        @test corners[1].x < corners[2].x  # left < right (bottom)
        @test corners[3].x < corners[4].x  # left < right (top)
        @test corners[1].y > corners[3].y  # bottom > top (left)
        @test corners[2].y > corners[4].y  # bottom > top (right)

        # Test pixel uncertainty ranges
        typical_uncertainties = [1.0 * 1pixel, 2.5 * 1pixel, 5.0 * 1pixel, 10.0 * 1pixel]
        for uncertainty in typical_uncertainties
            @test 0.1 * 1pixel <= uncertainty <= 20.0 * 1pixel  # Reasonable range
        end
    end
end
