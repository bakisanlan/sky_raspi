from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time 
import math
from pymavlink import mavutil
import dronekit
import geopy.distance
# from gpiozero import Servo
# import gpiozero
from time import sleep
import cv2      
import schedule
# from ServoControl import ServoControl
from camera_class import camera_class
#from get_wp_cmd import get_wp_cmd
import datetime
import numpy as np

def NED_to_lat_lon(ref_location, north, east):
    # convert north and east values to latitude and longitude offsets
    earth_radius = 6378137  # radius of the earth in meters
    lat_offset = north / earth_radius * (180 / math.pi)
    lon_offset = east / earth_radius * (180 / math.pi) / math.cos(ref_location.lat * math.pi / 180)

    # calculate new latitude and longitude
    obj_lat = ref_location.lat + lat_offset
    obj_lon = ref_location.lon + lon_offset

    return (obj_lat,obj_lon)    
