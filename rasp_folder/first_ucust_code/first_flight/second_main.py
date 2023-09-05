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
from obj_NED_rel_home import obj_NED_rel_home
from datetime import datetime
from geodetic_to_NED import geodetic_to_NED
import numpy as np
from NED_to_lat_lon import NED_to_lat_lon
from yaw_transformation import yaw_transformation
from get_bearing import get_bearing

#grav const
g=9.81

#servo param
my_GPIO1 = 23
my_GPIO2 = 24
correction = 0.50
max_pw = (2 + correction)/1000
min_pw = (1 - correction)/1000
isBack_bombed = False
isFront_bombed = False
back_servo_time = 0.50
front_servo_time = 2.01

#camera param
global out,fourcc,video,counter
counter = 1
video = cv2.VideoCapture(0)
frame_size = (640,480)
video_fps = 24
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output0.mp4', fourcc, video_fps, frame_size)
camera_alt = 40

#video saving function
def save_record():
    global out,fourcc,video,counter
    out.release()
    out = cv2.VideoWriter('output'+str(counter)+'.mp4', fourcc, video_fps, frame_size)
    counter += 1
    
#initiliazing camera saving schedule
schedule.every(45).seconds.do(save_record)

#bomb class
ServoControlBot = ServoControl(my_GPIO1, my_GPIO2, min_pw,max_pw)
#camera class
camera_bot = camera_class(video,fourcc,out)

#target class
###target loc
#target_loc_lat = 41.101577
#target_loc_long = 29.023327
#point1=LocationGlobalRelative(target_loc_lat,target_loc_long,40)


print("Trying to connect to the vehicle...")
vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)
print("Connected to the vehicle.")

cmds=vehicle.commands
cmds.download()
cmds.wait_ready()
cmds_list = list(cmds)

#area bound param
max_lat = 40.2330065
min_lat = 40.2311226
max_lon = 29.0092707
min_lon = 29.0074897


home_location = vehicle.home_location
if home_location == None:
    home_location = (40.2320412, 29.0083218,102)
#vehicle_location = vehicle.location.global_frame

#initiating camera recording
camera_bot.detect_x_y(frame_size,False,True)
print('Camera recording has been started')

fire_lat_lon = (40.2320359,29.0088736,20)
fire_lat_lon_rel = LocationGlobalRelative(fire_lat_lon[0],fire_lat_lon[1],fire_lat_lon[2])

tour = 0

def print_now():
    current_time = datetime.now().strftime("%H:%M:%S")
    print(current_time)

while True:
    camera_bot = camera_class(video,fourcc,out)
    detected_obj_px = camera_bot.detect_x_y(frame_size,False,True)  #return px
    
    if vehicle.mode == VehicleMode("AUTO") and (not isFront_bombed):
        if cmds.next == 6:
            tour = 1
        
        if (tour == 1) and (cmds.next == 1):
            
            vehicle.mode = VehicleMode('GUIDED')
            vehicle.simple_goto(fire_lat_lon_rel)
            
            while True: #1 
                camera_bot = camera_class(video,fourcc,out)
                detected_obj_px = camera_bot.detect_x_y(frame_size,False,True)  #return px  
                dist_from_drop_point = falling_algo(vehicle, vehicle.location.global_frame,fire_lat_lon_rel)[0]
                bearing_to_wp = get_bearing(vehicle.location.global_frame,fire_lat_lon_rel)
                if (dist_from_drop_point < back_servo_time*vehicle.groundspeed) and (abs(bearing_to_wp - math.degrees(vehicle.attitude.yaw)) < 30):
                                                            
                    if not isBack_bombed:
                        ServoControlBot.DropBackBomb()
                        isBack_bombed = True 
                        back_servo_time = front_servo_time
                        print_now()
                        print('Arka bomba birakildi')
                        
                    elif isBack_bombed:
                        ServoControlBot.DropFrontBomb()
                        isFront_bombed = True         
                        print_now()
                        print('On bomba birakildi')
                    cmds.next = 3
                    cmds.upload()
                    vehicle.mode = VehicleMode('AUTO')
                    break #1