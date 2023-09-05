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
#from get_wp_cmd import get_wp_cmd
import datetime
import numpy as np

def obj_NED_to_lat_lon(vehicle_location, north, east):
    # convert north and east values to latitude and longitude offsets
    earth_radius = 6378137  # radius of the earth in meters
    lat_offset = north / earth_radius * (180 / math.pi)
    lon_offset = east / earth_radius * (180 / math.pi) / math.cos(vehicle_location.lat * math.pi / 180)

    # calculate new latitude and longitude
    obj_lat = vehicle_location.lat + lat_offset
    obj_lon = vehicle_location.lon + lon_offset
    #print(new_latitude)
    # print new location
    #print(f"New location: ({new_latitude}, {new_longitude})")

    #return (new_latitude, new_longitude)
    return (obj_lat,obj_lon)    
# def CommandCreate(self,fire_location):
#     cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0,
#                 3, 0, 0,fire_location.lat, fire_location.lon,15)

#     return (cmd, fire_location)