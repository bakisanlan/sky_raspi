import math
import geopy

def falling_algo(vehicle, current_location, target_location):
    # Constants for falling algorithm
    rho = 1.225
    g = 9.80665
    m = 0.370
    Cd = 0.47
    radius = 0.044
    A = math.pi * radius**2
    
    # Retrieve altitude and speeds from the vehicle
    h = vehicle.location.global_relative_frame.alt
    if h < 0:
        h = 0
    groundSpeed = vehicle.groundspeed
    airSpeed = vehicle.airspeed
    windSpeed = groundSpeed - airSpeed
    
    # Handle edge cases to prevent division by zero
    if groundSpeed == 0:
        groundSpeed = 0.00001
    if airSpeed == 0:
        airSpeed = 0.00001
    if windSpeed == 0:
        windSpeed = 0.00001

    # Calculate parameters for the falling algorithm
    P = rho * Cd * A / (2 * m)
    k = P * m
    t = math.sqrt(abs(m/(g*k))) * math.acosh(math.exp(h*k/m))
    c1 = -1 / airSpeed
    c2 = -(-math.log(airSpeed) + windSpeed / airSpeed) / P
    
    # Calculate the range based on falling algorithm
    range = (math.log(P * t - c1) - c1 * windSpeed + P * t * windSpeed) / P + c2

    # Calculate the distance between current and target locations
    distance = geopy.distance.GeodesicDistance(
        (target_location.lat, target_location.lon),
        (current_location.lat, current_location.lon)
    ).meters
    
    # Return tuple containing calculated values
    return (distance - range, distance, range)
