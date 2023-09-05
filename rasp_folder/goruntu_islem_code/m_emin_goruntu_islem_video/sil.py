import cv2          
import numpy as np      #importing library
from time import sleep
import schedule
from camera_class import camera_class
from geodetic_to_NED import geodetic_to_NED
from new_obj_NED_rel_home import new_obj_NED_rel_home
import math

global out,fourcc,video,counter
counter = 1
video = cv2.VideoCapture('mehmet_e_video.mp4')
fps = video.get(cv2.CAP_PROP_FPS)
frame_num = 0
frame_size = (352,640)
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

home_location = (40.7567422 , 30.4085367,32)
camera_location = (40.7568895 , 30.4082231,47)
camera_bot.detect_x_y(frame_size,True,True)

def my_fun(detected_obj_px,euler):
    
        #vehicle_ned_rel_home = geodetic_to_NED((vehicle_location.alt,vehicle_location.lon, vehicle_location.lat),(home_location.lat,home_location.lon, home_location.alt))
        #vehicle_ned_rel_home = geodetic_to_NED((vehicle_location.alt,vehicle_location.lon, vehicle_location.lat),(40,20,0))
        vehicle_ned_rel_home = geodetic_to_NED(camera_location,home_location)
        obj_ned_rel_home = new_obj_NED_rel_home([euler[0],90-euler[1],euler[2]],detected_obj_px,list(vehicle_ned_rel_home),home_location[2])
        obj_ned_rel_vehicle = [obj_ned_rel_home[0]-vehicle_ned_rel_home[0], obj_ned_rel_home[1]-vehicle_ned_rel_home[1], obj_ned_rel_home[2]-vehicle_ned_rel_home[2]]

        return (vehicle_ned_rel_home, obj_ned_rel_home, obj_ned_rel_vehicle)


list_euler = [(0,30,0), (0,30,0),(0,30,0),(0,30,0),
              (0,30,0),(0,30,0),(0,30,0),(0,30,0),
              (11,30,0),(22,30,0),(33,30,0),(44,30,0),
              (33,30,0),(22,30,0),(11,30,0),(0,30,0),
              (0,30,0),(0,30,0),(0,30,0),(0,30,0),
              (0,30,0),(0,30,0),(0,30,0),(0,30,0)]
count = 0
while True:
    camera_bot = camera_class(video,fourcc,out)
    detected_obj_px = camera_bot.detect_x_y(frame_size,False,False)  #return px

    schedule.run_pending() #for camera saving schedule
    #print(vehicle.mode)
    camera_bot = camera_class(video,fourcc,out)
    frame_num += 1
    if frame_num % round(fps) ==0:
            
        detected_obj_px = camera_bot.detect_x_y(frame_size,True,False)  #return px
        #print_obj_loc(detected_obj)
        if detected_obj_px != None:
            ((vehicle_ned_rel_home, obj_ned_rel_home, obj_ned_rel_vehicle)) = my_fun(detected_obj_px,list_euler[count])
            print('Camera NED rel home = {}'.format(vehicle_ned_rel_home))
            print('Object NED rel home = {}'.format(obj_ned_rel_home))
            print('Object NED rel camera = {}'.format(obj_ned_rel_vehicle))
            print('--------------------------------------------------------------')

            count += 1
                

    if count == 11:
         break

