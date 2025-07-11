"""
Runway database management for pose estimation.

This module handles loading and managing runway specification data,
typically from Excel files containing runway dimensions and properties.
"""

using Unitful

"""
    RunwaySpec

Specification for a runway including dimensions and location properties.

# Fields
- `icao_code::String`: ICAO airport and runway identifier (e.g., "KJFK_04L")
- `length_m`: Runway length with units (typically meters)
- `width_m`: Runway width with units (typically meters)  
- `threshold_elevation_m`: Runway threshold elevation with units
- `true_bearing_deg`: Runway true bearing with units (typically degrees)

# Examples
```julia
# Create runway specification
runway = RunwaySpec(
    "KJFK_04L",
    3048.0u"m",    # 10,000 feet
    45.7u"m",      # 150 feet
    4.0u"m",       # 13 feet elevation
    40.0u"°"       # 40 degrees true bearing
)

# Access properties
println("Length: ", runway.length_m)
println("Width: ", runway.width_m)
```
"""
struct RunwaySpec{L, W, E, B}
    icao_code::String
    length_m::L
    width_m::W
    threshold_elevation_m::E
    true_bearing_deg::B
    
    function RunwaySpec(icao_code::String, length_m::L, width_m::W, 
                       threshold_elevation_m::E, true_bearing_deg::B) where {L, W, E, B}
        # Validate inputs
        if isempty(icao_code)
            throw(ArgumentError("ICAO code cannot be empty"))
        end
        
        if ustrip(length_m) <= 0
            throw(ArgumentError("Runway length must be positive"))
        end
        
        if ustrip(width_m) <= 0
            throw(ArgumentError("Runway width must be positive"))
        end
        
        # Normalize bearing to [0, 360) degrees
        bearing_deg = mod(ustrip(u"°", true_bearing_deg), 360.0)
        normalized_bearing = bearing_deg * unit(true_bearing_deg)
        
        new{L, W, E, B}(icao_code, length_m, width_m, threshold_elevation_m, normalized_bearing)
    end
end

"""
    get_runway_corners(runway_spec::RunwaySpec) -> StaticVector{4, WorldPoint}

Calculate the four corner points of a runway in world coordinates.

# Arguments
- `runway_spec::RunwaySpec`: Runway specification

# Returns
- `StaticVector` of 4 `WorldPoint`s representing runway corners

# Corner Ordering
1. Threshold left (near left)
2. Threshold right (near right)  
3. Far end left
4. Far end right

# Coordinate System
- Origin at runway threshold center
- X-axis: Along runway (positive towards far end)
- Y-axis: Across runway (positive towards right)
- Z-axis: Up from runway surface

# Examples
```julia
runway = RunwaySpec("TEST", 1000.0u"m", 50.0u"m", 0.0u"m", 0.0u"°")
corners = get_runway_corners(runway)

println("Threshold left: ", corners[1])
println("Threshold right: ", corners[2])
println("Far end left: ", corners[3])
println("Far end right: ", corners[4])
```
"""
function get_runway_corners(runway_spec::RunwaySpec)
    # Half-width for corner calculations
    half_width = runway_spec.width_m / 2
    
    # Define corners relative to threshold center
    corners = SA[
        WorldPoint(0.0u"m", -half_width, runway_spec.threshold_elevation_m),  # Threshold left
        WorldPoint(0.0u"m", half_width, runway_spec.threshold_elevation_m),   # Threshold right
        WorldPoint(runway_spec.length_m, -half_width, runway_spec.threshold_elevation_m),  # Far left
        WorldPoint(runway_spec.length_m, half_width, runway_spec.threshold_elevation_m)    # Far right
    ]
    
    return corners
end

"""
    validate_runway_spec(runway_spec::RunwaySpec) -> Bool

Validate that runway specification parameters are reasonable.

# Arguments
- `runway_spec::RunwaySpec`: Runway specification to validate

# Returns
- `Bool`: True if specification is valid

# Validation Criteria
- Length between 500m and 6000m
- Width between 15m and 100m
- Elevation between -100m and 5000m
- Bearing between 0° and 360°

# Examples
```julia
runway = RunwaySpec("KJFK_04L", 3048.0u"m", 45.7u"m", 4.0u"m", 40.0u"°")
is_valid = validate_runway_spec(runway)
println("Runway spec valid: ", is_valid)
```
"""
function validate_runway_spec(runway_spec::RunwaySpec)
    # Check length range (typical runway lengths)
    length_m = ustrip(u"m", runway_spec.length_m)
    if length_m < 500.0 || length_m > 6000.0
        return false
    end
    
    # Check width range (typical runway widths)
    width_m = ustrip(u"m", runway_spec.width_m)
    if width_m < 15.0 || width_m > 100.0
        return false
    end
    
    # Check elevation range (reasonable airport elevations)
    elevation_m = ustrip(u"m", runway_spec.threshold_elevation_m)
    if elevation_m < -100.0 || elevation_m > 5000.0
        return false
    end
    
    # Check bearing range
    bearing_deg = ustrip(u"°", runway_spec.true_bearing_deg)
    if bearing_deg < 0.0 || bearing_deg >= 360.0
        return false
    end
    
    return true
end
