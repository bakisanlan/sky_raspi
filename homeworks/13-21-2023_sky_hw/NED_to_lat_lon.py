
# from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
# import time 
# import math
# from pymavlink import mavutil
# import dronekit
# import geopy.distance
# from gpiozero import Servo
# import gpiozero
# from time import sleep
# import cv2      
# import schedule
# from ServoControl import ServoControl
# from camera_class import camera_class
# #from get_wp_cmd import get_wp_cmd
# import datetime
# import numpy as np
import navpy

def ned_to_lla(ned_coordinates, reference_lla):
    """
    Convert NED (North-East-Down) coordinates to LLA (Latitude-Longitude-Altitude) using navpy library.

    Parameters:
    - ned_coordinates: Tuple (north, east, down) representing NED coordinates.
    - reference_lla: Tuple (latitude, longitude, altitude) representing the reference point.

    Returns:
    Tuple (latitude, longitude, altitude) representing the converted LLA coordinates.
    """
    # Extract reference LLA coordinates
    lat_ref, lon_ref, alt_ref = reference_lla

    # Convert NED to LLA
    lla_coordinates = navpy.ned2lla(ned_coordinates, lat_ref, lon_ref, alt_ref, latlon_unit='deg', alt_unit='m', model='wgs84')

    return lla_coordinates



# Example usage:
ned_coordinates = (10000, 20000, -50)  # NED coordinates
reference_lla = (37.7749, -122.4194, 0)  # Reference LLA coordinates (San Francisco, CA)

lla_coordinates = ned_to_lla(ned_coordinates, reference_lla)
print("Converted LLA coordinates:", lla_coordinates)
