from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time 
import math
from pymavlink import mavutil
import dronekit
import geopy.distance
from gpiozero import Servo
import gpiozero
from time import sleep
import cv2      
import schedule
from ServoControl import ServoControl
from camera_class import camera_class
from falling_algo import falling_algo
#from get_wp_cmd import get_wp_cmd
import datetime
import numpy as np

def get_bearing(current_location, target_location):

    lat1, lon1 = current_location.lat,current_location.lon
    lat2, lon2 = target_location.lat,target_location.lon

    # Convert latitude and longitude to radians
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    # Calculate the bearing using the Haversine formula
    y = math.sin(lon2 - lon1) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1)
    bearing = math.degrees(math.atan2(y, x))

    # Normalize the bearing to a range of 0 to 360 degrees
    if bearing < 0:
        bearing += 360

    return bearing