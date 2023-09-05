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
import datetime
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
front_servo_time = 1.40

#camera param
global out,fourcc,video,counter
counter = 1
video = cv2.VideoCapture(0)
frame_size = (640,480)
video_fps = 24
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output0.mp4', fourcc, video_fps, frame_size)

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
max_lat = 30
min_lat = 30
max_lon = 30
min_lon = 30


home_location = vehicle.home_location
#vehicle_location = vehicle.location.global_frame

#initiating camera recording
camera_bot.detect_x_y(frame_size,False,True)
print('Camera recording has been started')

##################################################################
#####FUCTIONS THAT WILL HANDLE LATERRR ####
##################################################################
def print_obj_loc(detected_obj_px):
    if detected_obj_px != None:
        vehicle_location = vehicle.location.global_frame
        #vehicle_ned_rel_home = geodetic_to_NED((vehicle_location.alt,vehicle_location.lon, vehicle_location.lat),(home_location.lat,home_location.lon, home_location.alt))
        #vehicle_ned_rel_home = geodetic_to_NED((vehicle_location.alt,vehicle_location.lon, vehicle_location.lat),(40,20,0))
        vehicle_ned_rel_home = geodetic_to_NED(home_location,vehicle_location)
        obj_ned_rel_home = obj_NED_rel_home([math.degrees(vehicle.attitude.roll),math.degrees(vehicle.attitude.pitch),math.degrees(vehicle.attitude.yaw)],detected_obj_px,list(vehicle_ned_rel_home))
        obj_ned_rel_vehicle = [obj_ned_rel_home[0]-vehicle_ned_rel_home[0], obj_ned_rel_home[1]-vehicle_ned_rel_home[1], obj_ned_rel_home[2]-vehicle_ned_rel_home[2]]
        now = datetime.datetime.now()
        #print(obj_ned_rel_home)
        #print('girdim')
        time.sleep(0.2)
        #print('Home location {}'.format())            
        print('Vehicle NED location is {} /n '.format(vehicle_ned_rel_home))
        print('Object NED location is {} /n'.format(obj_ned_rel_home))
        print('Guessed Obj Location is = {} north {} east /n'.format(obj_ned_rel_vehicle[0],obj_ned_rel_vehicle[1]))
        print('roll: {} pitch: {} yaw{} /n'.format(math.degrees(vehicle.attitude.roll),math.degrees(vehicle.attitude.pitch),math.degrees(vehicle.attitude.yaw)))

def obj_px_to_obj_lat_lon(detected_obj_px):
    
        vehicle_location = vehicle.location.global_frame
        #vehicle_ned_rel_home = geodetic_to_NED((vehicle_location.alt,vehicle_location.lon, vehicle_location.lat),(home_location.lat,home_location.lon, home_location.alt))
        #vehicle_ned_rel_home = geodetic_to_NED((vehicle_location.alt,vehicle_location.lon, vehicle_location.lat),(40,20,0))
        vehicle_ned_rel_home = geodetic_to_NED(home_location,vehicle_location)
        obj_ned_rel_home = obj_NED_rel_home([math.degrees(vehicle.attitude.roll),math.degrees(vehicle.attitude.pitch),math.degrees(vehicle.attitude.yaw)],detected_obj_px,list(vehicle_ned_rel_home))
        obj_ned_rel_vehicle = [obj_ned_rel_home[0]-vehicle_ned_rel_home[0], obj_ned_rel_home[1]-vehicle_ned_rel_home[1], obj_ned_rel_home[2]-vehicle_ned_rel_home[2]]
        return NED_to_lat_lon(vehicle_location, obj_ned_rel_vehicle[0],obj_ned_rel_vehicle[1])

def get_obj_mean_lat_lon(detected_obj_px):
    
        #list_detected_obj_px = [detected_obj_px]
        list_obj_lat_lon = [obj_px_to_obj_lat_lon(detected_obj_px)]
        detected_obj_px = camera_bot.detect_x_y(frame_size,False,True)
        while detected_obj_px != None:
            list_obj_lat_lon.append(obj_px_to_obj_lat_lon(detected_obj_px))
            detected_obj_px = camera_bot.detect_x_y(frame_size,False,True)
            
        lat_values = [lat for lat, lon in list_obj_lat_lon]
        lon_values = [lon for lat, lon in list_obj_lat_lon]
        obj_lat_mean, obj_lon_mean = np.mean(lat_values), np.mean(lon_values)
        return (obj_lat_mean, obj_lon_mean)
    
def get_obj_mean_lat_lon_wp(detected_obj_px):
    
        #list_detected_obj_px = [detected_obj_px]
        list_obj_lat_lon = [obj_px_to_obj_lat_lon(detected_obj_px)]
        detected_obj_px = camera_bot.detect_x_y(frame_size,False,True)
        
        while (detected_obj_px != None):
            dist_vehicle_obj = distance_fun(obj_mean_lat_lon,vehicle.location.global_frame)
            if dist_vehicle_obj > 10:
                list_obj_lat_lon.append(obj_px_to_obj_lat_lon(detected_obj_px))
                detected_obj_px = camera_bot.detect_x_y(frame_size,False,True)
            
            lat_values = [lat for lat, lon in list_obj_lat_lon]
            lon_values = [lon for lat, lon in list_obj_lat_lon]
            obj_lat_mean, obj_lon_mean = np.mean(lat_values), np.mean(lon_values)
            
            return (obj_lat_mean, obj_lon_mean)
        return (list_obj_lat_lon[0][0],list_obj_lat_lon[0][1])
    
def distance_fun(target,reference):
    
    distance = geopy.distance.GeodesicDistance((target[0],target[1]),
                                            (reference.lat,
                                            reference.lon)).meters
    
    return abs(distance)
########################################################################################
########################################################################################

while True: #5
    print(1)
    #print('{} lat {} lon {} alt'.format(home_location.lat,home_location.lon,home_location.alt))
    #print('{} lat {} lon {} alt'.format(vehicle_location.lat,vehicle_location.lon,vehicle_location.alt))

    camera_bot = camera_class(video,fourcc,out)
    detected_obj_px = camera_bot.detect_x_y(frame_size,True,True)  #return px
    #print_obj_loc(detected_obj_px)
    
    schedule.run_pending() #for camera saving schedule
    
    if (vehicle.location.global_relative_frame.alt < 5) and (cmds.next > 5):
        out.release()
        video.release()
        
    else:
        print(2)

        
        if vehicle.mode == VehicleMode("AUTO") and (not isFront_bombed):
            print(3)

            #print(vehicle.mode)
            camera_bot = camera_class(video,fourcc,out)
            detected_obj_px = camera_bot.detect_x_y(frame_size,False,True)  #return px
            #print_obj_loc(detected_obj)
            
            #obje detect edilene kadar oto ucmaya devam et
            if detected_obj_px != None:
                print(4)
                
                obj_mean_lat_lon = get_obj_mean_lat_lon(detected_obj_px) 
                #obj_mean_NED_rel_vehicle_true_N = geodetic_to_NED(vehicle.location.global_frame, obj_mean_lat_lon)
                #obj_mean_NED_rel_vehicle_N = yaw_transformation(math.degrees(vehicle.attitude.yaw),obj_mean_NED_rel_vehicle_true_N)
                
                # tahmin edilen obje lokasyonu sinirlarin icinde mi
                if (obj_mean_lat_lon[0] < max_lat) and (obj_mean_lat_lon[0] > min_lat) and (obj_mean_lat_lon[1] < max_lon) and (obj_mean_lat_lon[1] > min_lon):
                    print(5)
                    while True: #4
                        print(6)
                        
                        #object detect edildikten sonra aradaki mesafe 50 metreden buyuk olana kadar devam et
                        dist_vehicle_obj = distance_fun(obj_mean_lat_lon,vehicle.location.global_frame)
                        if dist_vehicle_obj > 50:
                            print(7)
                            
                            #aradaki mesafe 50 metreden fazla olunca objeye yonel
                            #storing_next = cmds.next 
                            obj_mean_lat_lon_rel = LocationGlobalRelative(obj_mean_lat_lon[0],obj_mean_lat_lon[1],50)
                            vehicle.mode = VehicleMode('GUIDED')
                            vehicle.simple_goto(obj_mean_lat_lon_rel)
                            
                            #objeye yonlendiginde tekrar objeyi detect et
                            while True: #3
                                print(8)
                                camera_bot = camera_class(video,fourcc,out)
                                detected_obj_px = camera_bot.detect_x_y(frame_size,False,True)  #return px
                                dist_vehicle_obj_guess = distance_fun(obj_mean_lat_lon,vehicle.location.global_frame)

                                if detected_obj_px != None:
                                    print(9)
                                    #objeye giderken 10 metre kala tekrar eski oto missiona don ve ortalamasini al
                                    obj_mean_lat_lon_bef_wp = get_obj_mean_lat_lon_wp(detected_obj_px) 
                                    
                                    #objenin ustunde oldugunda ucaga en yakin wpden devam etmek
                                    temp_distance = 99999999
                                    cmd_next = 1
                                    for cmd in cmds_list:
                                        waypoint_location = LocationGlobal(cmd.x, cmd.y, cmd.z)
                                        waypoint_location = [waypoint_location.lat, waypoint_location.lon]
                                        close_wp_dist = distance_fun(waypoint_location, vehicle.location.global_frame)
                                        if close_wp_dist < temp_distance:
                                            temp_distance = close_wp_dist
                                            closest_cmd_next = cmd_next
                                        cmd_next += 1
                                    
                                    #Auto moda en yakin wpden devam etmek
                                    cmds.next = closest_cmd_next
                                    vehicle.mode = VehicleMode('AUTO')
        
                                    #ucaga en yakin wpden devam ettirip obje kameradan cikana kadar son location i almak
                                    camera_bot = camera_class(video,fourcc,out)
                                    detected_obj_px = camera_bot.detect_x_y(frame_size,False,True)  #return px
                                    if detected_obj_px != None:
                                        print(10)
                                        camera_bot = camera_class(video,fourcc,out)
                                        detected_obj_px = camera_bot.detect_x_y(frame_size,False,True)  #return px
                                        obj_mean_lat_lon_aft_wp = get_obj_mean_lat_lon(detected_obj_px) 
                                        final_obj_mean_lat = (obj_mean_lat_lon_bef_wp[0] +obj_mean_lat_lon_aft_wp[0]) / 2
                                        final_obj_mean_lon = (obj_mean_lat_lon_bef_wp[1] +obj_mean_lat_lon_aft_wp[1]) / 2 
                                        final_obj_mean_lat_lon = (final_obj_mean_lat,final_obj_mean_lon)
                                        
                                        #objeye yeteri kadar uzakliktaysa objeye git
                                        while True: #2
                                            print(11)
                                            
                                            dist_vehicle_obj = distance_fun(final_obj_mean_lat_lon,vehicle.location.global_frame)
                                            if dist_vehicle_obj > 80:
                                                print(12)
                                                
                                                final_obj_mean_lat_lon = LocationGlobalRelative(final_obj_mean_lat_lon[0],final_obj_mean_lat_lon[1],50)
                                                vehicle.mode = VehicleMode('GUIDED')
                                                vehicle.simple_goto(final_obj_mean_lat_lon)
                                                
                                                time_start = time.time()
                                                while True: #1
                                                    print(13)
                                                    dist_from_drop_point = falling_algo(vehicle, vehicle.location.global_frame,final_obj_mean_lat_lon)[0]
                                                    bearing_to_wp = get_bearing(vehicle.location.global_frame,final_obj_mean_lat_lon)
                                                    
                                                    
                                                    if (dist_from_drop_point < back_servo_time*vehicle.groundspeed) and (abs(bearing_to_wp - math.degrees(vehicle.attitude.yaw)) < 10):
                                                        print(14)
                                                        if not isBack_bombed:
                                                            print(15)
                                                            ServoControlBot.DropBackBomb()
                                                            isBack_bombed = True 
                                                            back_servo_time = front_servo_time
                                                        elif isBack_bombed:
                                                            ServoControlBot.DropFrontBomb()
                                                            isFront_bombed = True                                                   
                                                    
                                                        break #1
                                                    
                                                    # loiter yapmayi onlemek icin 30 saniye 
                                                    elif (time.time() - time_start) > 25:
                                                        break #1
                                                #ustteki loop kirilirsa auto ya al
                                                vehicle.mode = VehicleMode('AUTO')
                                                break #2
                                        break #3
                    
                                #eger tahmin edilen obje lokasyonuna giderken objeyi gormezse loopu ve en basa don
                                elif dist_vehicle_obj_guess < 5:
                                    vehicle.mode = VehicleMode('AUTO')
                                    break #3
                            break #4
        