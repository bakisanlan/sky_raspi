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
from obj_NED_to_lat_lon import obj_NED_to_lat_lon


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
schedule.every(60).seconds.do(save_record)

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
list_guess_obj_coor = [] 
cmds.download()
cmds.wait_ready()
list_cmd = list(cmds)


home_location = vehicle.home_location
#vehicle_location = vehicle.location.global_frame

#initiating camera recording
camera_bot.detect_x_y(frame_size,False,True)
print('Camera recording has been started')

#####FUCTIONS THAT WILL HANDLE LATERRR ####

def print_obj_loc(detected_obj_px):
    if detected_obj_px != None:
        vehicle_location = vehicle.location.global_frame
        #vehicle_ned_rel_home = geodetic_to_NED((vehicle_location.alt,vehicle_location.lon, vehicle_location.lat),(home_location.lat,home_location.lon, home_location.alt))
        #vehicle_ned_rel_home = geodetic_to_NED((vehicle_location.alt,vehicle_location.lon, vehicle_location.lat),(40,20,0))
        vehicle_ned_rel_home = geodetic_to_NED(home_location,vehicle_location)
        obj_ned_rel_home = obj_NED_rel_home(vehicle.location.global_relative_frame, [math.degrees(vehicle.attitude.roll),math.degrees(vehicle.attitude.pitch),math.degrees(vehicle.attitude.yaw)],detected_obj_px,list(vehicle_ned_rel_home))
        obj_ned_rel_vehicle = [obj_ned_rel_home[0]-vehicle_ned_rel_home[0], obj_ned_rel_home[1]-vehicle_ned_rel_home[1], vehicle_location.alt]
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
        vehicle_ned_rel_home = geodetic_to_NED(vehicle_location,home_location)
        obj_ned_rel_home = obj_NED_rel_home([math.degrees(vehicle.attitude.roll),math.degrees(vehicle.attitude.pitch),math.degrees(vehicle.attitude.yaw)],detected_obj_px,list(vehicle_ned_rel_home))
        obj_ned_rel_vehicle = [obj_ned_rel_home[0]-vehicle_ned_rel_home[0], obj_ned_rel_home[1]-vehicle_ned_rel_home[1], vehicle_location.alt]
        return obj_NED_to_lat_lon(vehicle_location, obj_ned_rel_vehicle[0],obj_ned_rel_vehicle[1])

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
######################


while True: 
    #print('{} lat {} lon {} alt'.format(home_location.lat,home_location.lon,home_location.alt))
    #print('{} lat {} lon {} alt'.format(vehicle_location.lat,vehicle_location.lon,vehicle_location.alt))

    camera_bot = camera_class(video,fourcc,out)
    detected_obj_px = camera_bot.detect_x_y(frame_size,True,False)  #return px
    print_obj_loc(detected_obj_px)
    
    schedule.run_pending() #for camera saving schedule
    
    if (vehicle.location.global_relative_frame.alt < 10) and (cmds.next > 2):
        out.release()
        video.release()
        
    else:
        if vehicle.mode!=VehicleMode("AUTO"):
            #print(vehicle.mode)
            camera_bot = camera_class(video,fourcc,out)
            detected_obj_px = camera_bot.detect_x_y(frame_size,False,True)  #return px
            #print_obj_loc(detected_obj)
            if detected_obj_px != None:
                obj_mean_lat_lon = get_obj_mean_lat_lon(detected_obj_px)
                
                #100 metre olana gitsin
                time.sleep(3)
                storing_next = cmds.next 
                cmds.clear()
                cmds.next =1
                cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0,
                  3, 0, 0,obj_mean_lat_lon[0],obj_mean_lat_lon[1] ,20)
                cmds.add(cmd)
                cmds.upload()
                vehicle.mode = VehicleMode('AUTO')
                while True:
                    if falling_algo(vehicle, vehicle.location.global_frame,(obj_mean_lat_lon[0],obj_mean_lat_lon[1]))[0] < 0.5*vehicle.groundspeed:
                        ServoControlBot.DropBackBomb()
                        isBack_bombed = True 
                        break
                cmds.clear()
                cmds.next = storing_next
                for i in list_cmd:
                    cmds.add(i)
                cmds.upload()
                vehicle.mode = VehicleMode('AUTO')
