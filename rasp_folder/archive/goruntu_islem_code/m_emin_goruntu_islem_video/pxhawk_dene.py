import cv2          
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command

import numpy as np      #importing library
from time import sleep
import schedule
from camera_class import camera_class
from geodetic_to_NED import geodetic_to_NED
from obj_NED_rel_home import obj_NED_rel_home
import math

global out,fourcc,video,counter
counter = 1
video = cv2.VideoCapture(0)
frame_num = 0
frame_size = (640,480)
video_fps = 24
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output0.mp4', fourcc, video_fps, frame_size)

def save_record():
    global out,fourcc,video,counter
    out.release()
    out = cv2.VideoWriter('output'+str(counter)+'.mp4', fourcc, video_fps, frame_size)
    counter += 1

schedule.every(60).seconds.do(save_record)
camera_bot = camera_class(video,fourcc,out)

print("Trying to connect to the vehicle...")
vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)
print("Connected to the vehicle.")

home_location = vehicle.home_location
camera_location = vehicle.location.global_frame
camera_bot.detect_x_y(frame_size,True,True)
list_mean = []
sum1=0
sum2=0
sum3=0

def my_fun(detected_obj_px):
    
    #vehicle_ned_rel_home = geodetic_to_NED((vehicle_location.alt,vehicle_location.lon, vehicle_location.lat),(home_location.lat,home_location.lon, home_location.alt))
    #vehicle_ned_rel_home = geodetic_to_NED((vehicle_location.alt,vehicle_location.lon, vehicle_location.lat),(40,20,0))
    vehicle_ned_rel_home = geodetic_to_NED(home_location,camera_location)
    obj_ned_rel_home = obj_NED_rel_home([math.degrees(vehicle.attitude.roll),math.degrees(vehicle.attitude.pitch),math.degrees(vehicle.attitude.yaw)],detected_obj_px,list(vehicle_ned_rel_home))
    obj_ned_rel_vehicle = [obj_ned_rel_home[0]-vehicle_ned_rel_home[0], obj_ned_rel_home[1]-vehicle_ned_rel_home[1], obj_ned_rel_home[2]-vehicle_ned_rel_home[2]]

    return (vehicle_ned_rel_home, obj_ned_rel_home, obj_ned_rel_vehicle)

i = 1
count = 0
while count < 10:
    camera_bot = camera_class(video,fourcc,out)
    detected_obj_px = camera_bot.detect_x_y(frame_size,False,False)  #return px
    camera_location = vehicle.location.global_frame

    schedule.run_pending() #for camera saving schedule
    #print(vehicle.mode)
    camera_bot = camera_class(video,fourcc,out)
            
    detected_obj_px = camera_bot.detect_x_y(frame_size,True,False)  #return px
    #print_obj_loc(detected_obj)
    if detected_obj_px != None:
        ((vehicle_ned_rel_home, obj_ned_rel_home, obj_ned_rel_vehicle)) = my_fun(detected_obj_px)
        print('Camera NED rel home = {}'.format(vehicle_ned_rel_home))
        print('Object NED rel home = {}'.format(obj_ned_rel_home))
        print('Object NED rel camera = {}'.format(obj_ned_rel_vehicle))
        print('--------------------------------------------------------------')

        count += 1

    #(vehicle_ned_rel_home, obj_ned_rel_home, obj_ned_rel_vehicle) = my_fun(detected_obj_px)
    #print('Object NED rel vehicle = {}'.format(obj_ned_rel_vehicle))
#     list_mean.append(vehicle_ned_rel_home)
#     #print(i)
#     i += 1
#     if i == 6000:
#         break

# for i in list_mean:
    
#     sum1 += i[0]
#     sum2 += i[1]
#     sum3 += i[2]

# mean1 = sum1/len(list_mean)
# mean2 = sum2/len(list_mean)
# mean3 = sum3/len(list_mean)

# print('({}, {}, {}) '.format(mean1,mean2,mean3))