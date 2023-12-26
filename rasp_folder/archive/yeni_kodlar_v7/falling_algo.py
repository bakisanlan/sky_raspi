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

def falling_algo(vehicle,current_location,target_location):
    
    rho = 1.225
    g = 9.80665
    m = 0.370
    Cd = 0.46
    radius = 0.025
    A = math.pi * radius**2
    
    h = vehicle.location.global_relative_frame.alt
    if h < 0:
        h = 0
    groundSpeed = vehicle.groundspeed
    airSpeed = vehicle.airspeed
    windSpeed = groundSpeed - airSpeed
    
    if groundSpeed == 0:
        groundSpeed = 0.00001
    if airSpeed == 0:
        airSpeed = 0.00001
    if windSpeed == 0:
        windSpeed =0

    P = rho * Cd * A / (2 * m)
    k = P * m
    t = math.sqrt(abs(m/(g*k))) * math.acosh(math.exp(h*k/m))
    c1 = -1 / airSpeed
    c2 = -(-math.log(airSpeed) + windSpeed / airSpeed) / P
    range = (math.log(P * t - c1) - c1 * windSpeed + P * t * windSpeed) / P + c2

    distance = geopy.distance.GeodesicDistance((target_location.lat,target_location.lon),
                                            (current_location.lat,
                                            current_location.lon)).meters
    
    return (distance-range, distance, range)