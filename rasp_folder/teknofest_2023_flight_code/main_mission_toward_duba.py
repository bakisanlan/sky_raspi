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
import random



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


print("Trying to connect to the vehicle...")
vehicle = connect("127.0.0.1:14550", wait_ready=True)
print("Connected to the vehicle.")

#area bound param
max_lat = 40.2330065
min_lat = 40.2311226
max_lon = 29.0092707
min_lon = 29.0074897


current_lat = vehicle.location.global_frame.lat
current_lon = vehicle.location.global_frame.lon

waypoints = [
    (40.2311226, 29.0092707),
    (40.2330065, 29.0092707),
    (40.2330065, 29.0074897),
    (40.2311226, 29.0074897)
]


home_location = vehicle.home_location
if home_location == None:
    home_location = (40.2320412, 29.0083218,100)
#vehicle_location = vehicle.location.global_frame

# if vehicle.mode == VehicleMode('AUTO'):
#     vehicle.mode = VehicleMode('MANUAL')

#initiating camera recording
camera_bot.detect_x_y(frame_size,False,True)
print('Camera recording has been started')

##################################################################
#####FUCTIONS THAT WILL HANDLE LATERRR ####
##################################################################

def takeoff(irtifa):
    while vehicle.is_armable is not True:
        print("İHA arm edilebilir durumda değil.")
        time.sleep(1)

    print("İHA arm edilebilir.")

    vehicle.mode = VehicleMode("GUIDED")

    vehicle.armed = True

    while vehicle.armed is not True:
        print("İHA arm ediliyor...")
        time.sleep(0.5)

    print("İHA arm edildi.")

def get_distance_metres(aLocation1, aLocation2):
   
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def goto(lat, lon, gotoFunction=vehicle.simple_goto):

    global targetDistance
    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = LocationGlobalRelative(lat, lon,40)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        global remainingDistance
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.2: #Just below target, in case of undershoot.
            print("Reached target")
            break
        time.sleep(2)

def gorev_ekle():
    global cmds
    cmds = vehicle.commands
    global mission
    mission = []

 
    cmds.download()
    cmds.wait_ready()
    vehicle.commands.clear()
    time.sleep(1)

    # TAKEOFF
    mission.append((Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 40)))

    # WAYPOINT
    mission.append((Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 10, 0, 0, 0, 40.2311226, 29.0092707, 40)))
    mission.append((Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 10, 0, 0, 0, 40.2330065, 29.0092707, 40)))
    mission.append((Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 10, 0, 0, 0, 40.2330065, 29.0074897, 40)))
    mission.append((Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 10, 0, 0, 0, 40.2311226, 29.0074897, 40)))
    
    mission.append((Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_JUMP,0, 0, 2, -1, 0, 0, 0, 0, 0)))

    # Upload the mission
    for cmd in mission:
        vehicle.commands.add(cmd)
    vehicle.commands.upload()
    print("Commands are uploading.")


def print_obj_loc(detected_obj_px):
    if detected_obj_px != None:
        vehicle_location = vehicle.location.global_frame
        #vehicle_ned_rel_home = geodetic_to_NED((vehicle_location.alt,vehicle_location.lon, vehicle_location.lat),(home_location.lat,home_location.lon, home_location.alt))
        #vehicle_ned_rel_home = geodetic_to_NED((vehicle_location.alt,vehicle_location.lon, vehicle_location.lat),(40,20,0))
        vehicle_ned_rel_home = geodetic_to_NED(home_location,vehicle_location)
        obj_ned_rel_home = obj_NED_rel_home([math.degrees(vehicle.attitude.roll),math.degrees(vehicle.attitude.pitch),math.degrees(vehicle.attitude.yaw)],detected_obj_px,list(vehicle_ned_rel_home))
        obj_ned_rel_vehicle = [obj_ned_rel_home[0]-vehicle_ned_rel_home[0], obj_ned_rel_home[1]-vehicle_ned_rel_home[1], obj_ned_rel_home[2]-vehicle_ned_rel_home[2]]
        #now = datetime.datetime.now()
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
        while True:
            time_start = time.time()
            while (time.time() - time_start) < 0.2: 
                detected_obj_px = camera_bot.detect_x_y(frame_size,False,True)
                while detected_obj_px != None:
                    list_obj_lat_lon.append(obj_px_to_obj_lat_lon(detected_obj_px))
                    detected_obj_px = camera_bot.detect_x_y(frame_size,False,True)
                    time_start = time.time()    
            break
        if len(list_obj_lat_lon) == 1:
            return (list_obj_lat_lon[0][0],list_obj_lat_lon[0][1])   
        else:
            lat_values = [lat for lat, lon in list_obj_lat_lon]
            lon_values = [lon for lat, lon in list_obj_lat_lon]
            obj_lat_mean, obj_lon_mean = np.mean(lat_values), np.mean(lon_values)
            obj_lat_std, obj_lon_std = np.std(lat_values), np.std(lon_values)
            #filtering of pos
            threshold = 2
            lat_z_scores = [(lat - obj_lat_mean) / obj_lat_std for lat in lat_values]
            lon_z_scores = [(lon - obj_lon_mean) / obj_lon_std for lon in lon_values]
            filtered_data = [(x, y) for (x, y), x_z, y_z in zip(list_obj_lat_lon, lat_z_scores, lon_z_scores) if abs(x_z) <= threshold and abs(y_z) <= threshold]
            filtered_lat_values = [lat for lat, lon in filtered_data]
            filtered_lon_values = [lon for lat, lon in filtered_data]
            filtered_obj_lat_mean = np.mean(filtered_lat_values)
            filtered_obj_lon_mean = np.mean(filtered_lon_values)
            return (filtered_obj_lat_mean, filtered_obj_lon_mean)

def get_obj_mean_lat_lon_wp(detected_obj_px,obj_mean_lat_lon):
        #list_detected_obj_px = [detected_obj_px]
        list_obj_lat_lon = [obj_px_to_obj_lat_lon(detected_obj_px)]
        while True:
            time_start = time.time()
            while (time.time() - time_start) < 0.2:
                detected_obj_px = camera_bot.detect_x_y(frame_size,False,True)
                while (detected_obj_px != None):
                    dist_vehicle_obj = distance_fun(obj_mean_lat_lon,vehicle.location.global_frame)
                    if dist_vehicle_obj > 10:
                        list_obj_lat_lon.append(obj_px_to_obj_lat_lon(detected_obj_px))
                        detected_obj_px = camera_bot.detect_x_y(frame_size,False,True)
                        time_start = time.time()          
            break
        if len(list_obj_lat_lon) == 1:
            return (list_obj_lat_lon[0][0],list_obj_lat_lon[0][1])   
        else:
            lat_values = [lat for lat, lon in list_obj_lat_lon]
            lon_values = [lon for lat, lon in list_obj_lat_lon]
            obj_lat_mean, obj_lon_mean = np.mean(lat_values), np.mean(lon_values)  
            obj_lat_std, obj_lon_std = np.std(lat_values), np.std(lon_values)
            #filtering of pos
            threshold = 2
            lat_z_scores = [(lat - obj_lat_mean) / obj_lat_std for lat in lat_values]
            lon_z_scores = [(lon - obj_lon_mean) / obj_lon_std for lon in lon_values]
            filtered_data = [(x, y) for (x, y), x_z, y_z in zip(list_obj_lat_lon, lat_z_scores, lon_z_scores) if abs(x_z) <= threshold and abs(y_z) <= threshold]
            filtered_lat_values = [lat for lat, lon in filtered_data]
            filtered_lon_values = [lon for lat, lon in filtered_data]
            filtered_obj_lat_mean = np.mean(filtered_lat_values)
            filtered_obj_lon_mean = np.mean(filtered_lon_values)
            return (filtered_obj_lat_mean, filtered_obj_lon_mean)                
    
def distance_fun(target,reference):
    
    distance = geopy.distance.GeodesicDistance((target[0],target[1]),
                                            (reference.lat,
                                            reference.lon)).meters
    return abs(distance)

def print_now():
    current_time = datetime.now().strftime("%H:%M:%S")
    print(current_time)
########################################################################################
########################################################################################
    

takeoff(20)
gorev_ekle()
print('Auto mission is starting...')

while vehicle.mode != VehicleMode("AUTO"):
    vehicle.mode = VehicleMode("AUTO")
    time.sleep(0.5)

cmds.next = 0

count = 0

while True:

    camera_bot = camera_class(video,fourcc,out)
    detected_obj_px = camera_bot.detect_x_y(frame_size,True,True)
    
    if vehicle.commands.next == 5:
        count = 1
    
    next_waypoint = cmds.next

    print(f"Next cmds {next_waypoint} and vehicle mode {vehicle.mode.name}")
    time.sleep(1)


    if next_waypoint == 4 and count == 1:
        camera_bot = camera_class(video,fourcc,out)
        detected_obj_px = camera_bot.detect_x_y(frame_size,False,True)
        time.sleep(2)
        print("Cut the loop ")

        while vehicle.mode != VehicleMode("GUIDED"):
            vehicle.mode = VehicleMode("GUIDED")
            time.sleep(0.5)
    
        print(vehicle.mode)
        

        #obje detect edilene kadar oto ucmaya devam et
        if detected_obj_px != None:
            print('Ates tespit edildi')
            obj_mean_lat_lon = get_obj_mean_lat_lon(detected_obj_px) 
            #obj_mean_NED_rel_vehicle_true_N = geodetic_to_NED(vehicle.location.global_frame, obj_mean_lat_lon)
            #obj_mean_NED_rel_vehicle_N = yaw_transformation(math.degrees(vehicle.attitude.yaw),obj_mean_NED_rel_vehicle_true_N)
            
            # tahmin edilen obje lokasyonu sinirlarin icinde mi
            if (obj_mean_lat_lon[0] < max_lat) and (obj_mean_lat_lon[0] > min_lat) and (obj_mean_lat_lon[1] < max_lon) and (obj_mean_lat_lon[1] > min_lon):
                print('Objenin ilk tahmini koordinatlari')
                print('lat: {}, lon {}'.format(obj_mean_lat_lon[0],obj_mean_lat_lon[1]))                    
                
                print_now()
                while True: #4

                    camera_bot = camera_class(video,fourcc,out)
                    detected_obj_px = camera_bot.detect_x_y(frame_size,False,True) 
                    #object detect edildikten sonra aradaki mesafe 50 metreden buyuk olana kadar devam et
                    dist_vehicle_obj = distance_fun(obj_mean_lat_lon,vehicle.location.global_frame)
                    print('Atese olan uzaklik: {}'.format(dist_vehicle_obj)) 
                    print_now()


                    if dist_vehicle_obj > 125:
                            print('Atese 125 metreden uzak oldu')
                    
                            obj_mean_lat_lon_rel = LocationGlobalRelative(obj_mean_lat_lon[0],obj_mean_lat_lon[1],camera_alt)
                            print('Objeye goto ile yoneliyor')
                            goto(obj_mean_lat_lon[0],obj_mean_lat_lon[1])
                           
                            #objeye yonlendiginde tekrar objeyi detect et
                            while True: #3

                                print_now()
                                camera_bot = camera_class(video,fourcc,out)
                                detected_obj_px = camera_bot.detect_x_y(frame_size,False,True)  #return px
                                obj_mean_lat_lon = (obj_mean_lat_lon_rel.lat, obj_mean_lat_lon_rel.lon)
                                dist_vehicle_obj_guess = distance_fun(obj_mean_lat_lon,vehicle.location.global_frame)

                                if detected_obj_px != None:
                                    #objeye giderken 10 metre kala tekrar eski oto missiona don ve ortalamasini al
                                    obj_mean_lat_lon_bef_wp = get_obj_mean_lat_lon_wp(detected_obj_px,obj_mean_lat_lon) 
                                    
                                    #objenin ustunde oldugunda ucaga en yakin wpden devam etmek
                                    temp_distance = 99999999
                                    
                                    cmds.next = 5
                                    cmds.upload()
                                    time.sleep(0.5)

                                    while vehicle.mode != VehicleMode("AUTO"):
                                        vehicle.mode = VehicleMode("AUTO")
                                        time.sleep(0.5)


                                    print('4. Wpye gidiyor..')

                                    camera_bot = camera_class(video,fourcc,out)
                                    time_start = time.time()
                                    while (time.time() - time_start) < 2:  #!
                                        detected_obj_px = camera_bot.detect_x_y(frame_size,False,True)  #return px  
                                        if detected_obj_px != None:   
                                            obj_mean_lat_lon_aft_wp = get_obj_mean_lat_lon(detected_obj_px) 
                                            final_obj_mean_lat = (obj_mean_lat_lon_bef_wp[0] +obj_mean_lat_lon_aft_wp[0]) / 2
                                            final_obj_mean_lon = (obj_mean_lat_lon_bef_wp[1] +obj_mean_lat_lon_aft_wp[1]) / 2 
                                            final_obj_mean_lat_lon = (final_obj_mean_lat,final_obj_mean_lon)
    
                     
                                            while True: #2
                                                dist_vehicle_obj = distance_fun(final_obj_mean_lat_lon,vehicle.location.global_frame)
                                                print('Atese olan uzaklik: {} '.format(dist_vehicle_obj))
                                                if dist_vehicle_obj > 65:
                                                    print('Aradaki mesafe 65 metreyi gecti, atese gidiyor')
                                                    
                                                    final_obj_mean_lat_lon = LocationGlobalRelative(final_obj_mean_lat_lon[0],final_obj_mean_lat_lon[1],camera_alt)
                                                    while vehicle.mode != VehicleMode("GUIDED"):
                                                        vehicle.mode = VehicleMode("GUIDED")
                                                        time.sleep(0.5)

                                                    goto(final_obj_mean_lat_lon.lat,final_obj_mean_lat_lon.lon)
                                                    time_start = time.time()
                                                    while True: #1

                                                        #final_obj_mean_lat_lon = LocationGlobalRelative(final_obj_mean_lat_lon[0],final_obj_mean_lat_lon[1],camera_alt)
                                                        dist_from_drop_point = falling_algo(vehicle, vehicle.location.global_frame,final_obj_mean_lat_lon)[0]
                                                        bearing_to_wp = get_bearing(vehicle.location.global_frame,final_obj_mean_lat_lon)
                                                        print_now()
                                                        print('Dist from droppoint is: {}, Bearing: {}'.format(dist_from_drop_point,bearing_to_wp))
                                                        if (dist_from_drop_point <= 0) and (abs(bearing_to_wp - math.degrees(vehicle.attitude.yaw)) < 10):
                                                            
                                                            if not isBack_bombed:
                                                                # ServoControlBot.DropBackBomb()
                                                                isBack_bombed = True 
                                                                back_servo_time = front_servo_time
                                                                print_now()
                                                                print('Arka bomba birakildi')
                                                            elif isBack_bombed:
                                                                # ServoControlBot.DropFrontBomb()
                                                                isFront_bombed = True         
                                                                print_now()
                                                                print('On bomba birakildi')                                          

                                                            print('otoya donuyor')
                                                            break #1
                                                        
                                                        # loiter yapmayi onlemek icin 20 saniye 
                                                        elif (time.time() - time_start) > 20:
                                                            print('zaman asimina ugradi otoya donuyor')
                                                            break #1
                                                        
                                                    #ustteki loop kirilirsa auto ya al
                                                    cmds.next = 4
                                                    cmds.upload()
                                                    while vehicle.mode != VehicleMode("AUTO"):
                                                        vehicle.mode = VehicleMode("AUTO")
                                                        time.sleep(0.5)
                                                    break #2
                                            break #! 

                                    while vehicle.mode != VehicleMode("AUTO"):
                                        vehicle.mode = VehicleMode("AUTO")
                                        time.sleep(0.5) 
                                    print('2 saniye icinde objeyi tespit edemedi, otoya donuyor')
                                    break #3 yeni         
                                        #objeye yeteri kadar uzakliktaysa objeye git
                                        #break #3 eski
                    
                                #eger tahmin edilen obje lokasyonuna giderken objeyi gormezse loopu ve en basa don
                                elif dist_vehicle_obj_guess < 5:
                                    print('Obje giderken goremedi, autoya dondu')
                                    
                                    while vehicle.mode != VehicleMode("AUTO"):
                                        vehicle.mode = VehicleMode("AUTO")
                                        time.sleep(0.5)
                                    break #3
                            break #4
                else:
                    print('Tespit edilen obje sinirin disinda, otoya donuyor') 
                                                                        
             
                    


                    