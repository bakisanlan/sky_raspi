from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time 
import math
from pymavlink import mavutil
# import dronekit
import geopy.distance
# from gpiozero import Servo
#import gpiozero
from time import sleep
import cv2      
import schedule
#from ServoControl import ServoControl
from camera_class import camera_class
#from falling_algo import falling_algo
# from get_wp_cmd import get_wp_cmd
from obj_NED_rel_home import obj_NED_rel_home
from datetime import datetime
from geodetic_to_NED import geodetic_to_NED
import numpy as np
from NED_to_lat_lon import NED_to_lat_lon
from yaw_transformation import yaw_transformation
from get_bearing import get_bearing
from math import radians, sin, cos, sqrt,atan2
from mission import DroneController
from util import Utility



print("Trying to connect to the vehicle...")
vehicle = connect("127.0.0.1:14550", wait_ready=True)
print("Connected to the vehicle.")


missions = DroneController(vehicle)
util = Utility(vehicle)

#grav const
# g=9.81

# #servo param
# my_GPIO1 = 23
# my_GPIO2 = 24
# correction = 0.50
# max_pw = (2 + correction)/1000
# min_pw = (1 - correction)/1000
isBack_bombed = False
isFront_bombed = False
back_servo_time = 0.5
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
#ServoControlBot = ServoControl(my_GPIO1, my_GPIO2, min_pw,max_pw)
#camera class
camera_bot = camera_class(video,fourcc,out)

#target class
###target loc
#target_loc_lat = 41.101577
#target_loc_long = 29.023327
#point1=LocationGlobalRelative(target_loc_lat,target_loc_long,40)
#area bound param
max_lat = 40.2330065
min_lat = 40.2311226
max_lon = 29.0092707
min_lon = 29.0074897

current_lat = vehicle.location.global_frame.lat
current_lon = vehicle.location.global_frame.lon

home_location = vehicle.home_location
if home_location == None:
    home_location = (40.22948110, 29.00889869,100)
#vehicle_location = vehicle.location.global_frame

# if vehicle.mode == VehicleMode('AUTO'):
#     vehicle.mode = VehicleMode('MANUAL')

#initiating camera recording
camera_bot.detect_x_y(frame_size,False,True)
print('Camera recording has been started')


# WAYPOINTS
waypoints = [
    (40.2311226, 29.0092707, 40),
    (40.2330065, 29.0092707, 40),
    (40.2330065, 29.0074897, 40),
    (40.2311226, 29.0074897, 40)
]
takeoff_alt = 40
missions.arm()
missions.add_mission(waypoints,takeoff_alt)
print('Auto mission is starting...')

while vehicle.mode != VehicleMode("AUTO"):
    vehicle.mode = VehicleMode("AUTO")
    time.sleep(0.5)

count = 0

while True:

    camera_bot = camera_class(video,fourcc,out)
    detected_obj_px = camera_bot.detect_x_y(frame_size,True,True)
    
    if vehicle.commands.next == 5:
        count = 1
    
    next_waypoint = missions.cmds.next

    print(f"Next cmds {next_waypoint} and vehicle mode {vehicle.mode.name}")
    time.sleep(1)
        

    #continue auto flying until object is detected
    if detected_obj_px != None and (count == 1):
        print('Fire detected')
        obj_mean_lat_lon = util.get_obj_mean_lat_lon(detected_obj_px) 
        #obj_mean_NED_rel_vehicle_true_N = geodetic_to_NED(vehicle.location.global_frame, obj_mean_lat_lon)
        #obj_mean_NED_rel_vehicle_N = yaw_transformation(math.degrees(vehicle.attitude.yaw),obj_mean_NED_rel_vehicle_true_N)
        
        # Is the estimated object location within the limits?
        if (obj_mean_lat_lon[0] < max_lat) and (obj_mean_lat_lon[0] > min_lat) and (obj_mean_lat_lon[1] < max_lon) and (obj_mean_lat_lon[1] > min_lon):
            print('First estimated coordinates of the object')
            print('lat: {}, lon {}'.format(obj_mean_lat_lon[0],obj_mean_lat_lon[1]))                    
            
            util.print_now()
            while True: #4
                
                camera_bot = camera_class(video,fourcc,out)
                detected_obj_px = camera_bot.detect_x_y(frame_size,False,True) 
                #After the object is detected, continue until the distance is greater than 50 meters.
                dist_vehicle_obj = util.distance_fun(obj_mean_lat_lon,vehicle.location.global_frame)
                print('Distance from fire: {}'.format(dist_vehicle_obj)) 
                util.print_now()

                if next_waypoint == 4:
                        
                    camera_bot = camera_class(video,fourcc,out)
                    detected_obj_px = camera_bot.detect_x_y(frame_size,False,True)

                    print("Cut the loop and uav directs to the object.")

                    while vehicle.mode != VehicleMode("GUIDED"):
                        vehicle.mode = VehicleMode("GUIDED")
                        time.sleep(0.5)
                
                    print(vehicle.mode)
            
                    obj_mean_lat_lon_rel = LocationGlobalRelative(obj_mean_lat_lon[0],obj_mean_lat_lon[1],camera_alt)
                    print('Directs to the object with goto.')
                    missions.goto(obj_mean_lat_lon_rel)
                    
                    #Detect the object again when directed to the object
                    while True: #3

                        util.print_now()
                        camera_bot = camera_class(video,fourcc,out)
                        detected_obj_px = camera_bot.detect_x_y(frame_size,False,True)  #return px
                        obj_mean_lat_lon = (obj_mean_lat_lon_rel.lat, obj_mean_lat_lon_rel.lon)
                        dist_vehicle_obj_guess = util.distance_fun(obj_mean_lat_lon,vehicle.location.global_frame)

                        if detected_obj_px != None:
                            #When you are 10 meters away from the object, go back to the old auto mission and take the average.
                            ## !!!!
                            obj_mean_lat_lon_bef_wp = util.get_obj_mean_lat_lon_wp(detected_obj_px,obj_mean_lat_lon) 
                    
                            missions.cmds.next = 5
                            missions.cmds.upload()
                            time.sleep(0.5)

                            while vehicle.mode != VehicleMode("AUTO"):
                                vehicle.mode = VehicleMode("AUTO")
                                time.sleep(0.5)


                            print('Going to Wp4 ...')

                            camera_bot = camera_class(video,fourcc,out)
                            time_start = time.time()
                            while (time.time() - time_start) < 2:  #!
                                detected_obj_px = camera_bot.detect_x_y(frame_size,False,True)  #return px  
                                if detected_obj_px != None:   
                                    obj_mean_lat_lon_aft_wp = util.get_obj_mean_lat_lon(detected_obj_px) 
                                    final_obj_mean_lat = (obj_mean_lat_lon_bef_wp[0] +obj_mean_lat_lon_aft_wp[0]) / 2
                                    final_obj_mean_lon = (obj_mean_lat_lon_bef_wp[1] +obj_mean_lat_lon_aft_wp[1]) / 2 
                                    final_obj_mean_lat_lon = (final_obj_mean_lat,final_obj_mean_lon)

                
                                    while True: #2
                                        dist_vehicle_obj = util.distance_fun(final_obj_mean_lat_lon,vehicle.location.global_frame)
                                        print('Distance from fire: {} '.format(dist_vehicle_obj))
                                        if missions.cmds.next == 2:
                                            
                                            
                                            final_obj_mean_lat_lon = LocationGlobalRelative(final_obj_mean_lat_lon[0],final_obj_mean_lat_lon[1],camera_alt)
                                            while vehicle.mode != VehicleMode("GUIDED"):
                                                vehicle.mode = VehicleMode("GUIDED")
                                                time.sleep(0.5)

                                            missions.goto(final_obj_mean_lat_lon) 
                                            time_start = time.time()
                                            while True: #1

                                                #final_obj_mean_lat_lon = LocationGlobalRelative(final_obj_mean_lat_lon[0],final_obj_mean_lat_lon[1],camera_alt)
                                                dist_from_drop_point = util.falling_algo(vehicle, vehicle.location.global_frame,final_obj_mean_lat_lon)[0]
                                                bearing_to_wp = get_bearing(vehicle.location.global_frame,final_obj_mean_lat_lon)
                                                util.print_now()
                                                print('Dist from droppoint is: {}, Bearing: {}'.format(dist_from_drop_point,bearing_to_wp))
                                                if (dist_from_drop_point <= 0) and (abs(bearing_to_wp - math.degrees(vehicle.attitude.yaw)) < 10):
                                                    
                                                    if not isBack_bombed:
                                                        # ServoControlBot.DropBackBomb()
                                                        isBack_bombed = True 
                                                        back_servo_time = front_servo_time
                                                        util.print_now()
                                                        print('The rear bomb was dropped.')
                                                    elif isBack_bombed:
                                                        # ServoControlBot.DropFrontBomb()
                                                        isFront_bombed = True         
                                                        util.print_now()
                                                        print('The front bomb was dropped.')                                          

                                                    print('Turning back to Auto mode.')

                                                    while vehicle.mode != VehicleMode("AUTO"):
                                                        vehicle.mode = VehicleMode("AUTO")
                                                        time.sleep(0.5)

                                                    print("UAV is going to home location.")
                                                    missions.cmds.commands.clear()
                                                    time.sleep(0.5)
                                                    missions.land()
                                                    vehicle.close()

                                                    break #1
                                                
                                                # 20 seconds to avoid loitering
                                                elif (time.time() - time_start) > 20:
                                                    print('Timed out, returns to Auto mode')
                                                    break #1
                                                
                                            #If the loop above is broken, switch to Auto mode
                                            missions.cmds.next = 4
                                            missions.cmds.upload()
                                            while vehicle.mode != VehicleMode("AUTO"):
                                                vehicle.mode = VehicleMode("AUTO")
                                                time.sleep(0.5)
                                            break #2
                                    break #! 

                            while vehicle.mode != VehicleMode("AUTO"):
                                vehicle.mode = VehicleMode("AUTO")
                                time.sleep(0.5) 
                            print("Couldn't detect the object within 2 seconds, goes back to auto")
                            break #3 yeni         
                                #If it is far enough away from the object, go to the object
                              
                        #If it does not see the object while going to the predicted object location, loop and go back to the beginning.
                        elif dist_vehicle_obj_guess < 20:
                            print("UAV couldn't see the object during flight, so UAV switched to Auto mode.")
                            
                            while vehicle.mode != VehicleMode("AUTO"):
                                vehicle.mode = VehicleMode("AUTO")
                                time.sleep(0.5)
                            break #3
                    break #4
        else:
            print('The detected object is outside the boundary, it turns into auto') 

        
                                                                        
             
                    

