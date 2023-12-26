import math
import numpy as np
from dronekit import  connect, VehicleMode, LocationGlobalRelative,Command
import time
from pymavlink import mavutil
import geopy.distance
import cv2
from geodetic_to_NED import geodetic_to_NED

def yaw_transformation(yaw_angle_deg,point):
    
    yaw = math.radians(yaw_angle_deg)  # radians
    point_vec = np.array([point[0], point[1], point[2]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    (north, east, down) = np.dot(Rz,point_vec)
    
    return (north, east, down)
