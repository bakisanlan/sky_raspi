from dronekit import Command, connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import time
from pymavlink import mavutil
import random
import math 

drone = connect('127.0.0.1:14550', wait_ready=True)

min_lat = -35.36265574
max_lat = -35.36085911765
min_lon = 149.16292425
max_lon = 149.16512720

random_lat = random.uniform(min_lat, max_lat)
random_lon = random.uniform(min_lon, max_lon)

# for landing
random_lat_l = random.uniform(min_lat, max_lat)
random_lon_l = random.uniform(min_lon, max_lon)


def take_off(altitude):
    while drone.is_armable is not True:
        print(" Drone is not armable.")
        time.sleep(1)

    print("Drone is armable.")

    drone.mode = VehicleMode("GUIDED")

    drone.armed = True

    while drone.armed is not True:
        print("Drone is arming.")
        time.sleep(1)

    print("Drone armed.")
    drone.simple_takeoff(altitude)

    while drone.location.global_relative_frame.alt >= altitude*0.9:
        print("Drone is rising. ")
        time.sleep(1)



def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # Radius of the Earth in meters
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance

def calculate_nearest_corner(random_lat,random_lon, min_lat, max_lat, min_lon, max_lon):

    global flag
    flag = 1

    corner1 = (min_lat, max_lon)
    corner2 = (max_lat, max_lon)
    corner3 = (max_lat, min_lon)
    corner4 = (min_lat, min_lon)

    corners = [corner1, corner2, corner3, corner4]


    shortest_distance = None
    nearest_corner = None

    for corner in corners:
        distance = haversine(random_lat, random_lon, corner[0], corner[1])
        if shortest_distance is None or distance < shortest_distance:
            shortest_distance = distance
            nearest_corner = corner


    for i in corners:

        if nearest_corner == i:
            break

        else:
            flag += 1
    
    if nearest_corner is not None:
        print("Nearest corner is:", nearest_corner)
        print("Shortest distance is:", shortest_distance)
        print ("Nearest corner is: ",flag)


    return nearest_corner


nearest_corner_lat, nearest_corner_lon = calculate_nearest_corner(random_lat,random_lon, min_lat, max_lat, min_lon, max_lon)


def get_distance_metres(aLocation1, aLocation2):
   
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def goto(dNorth, dEast, gotoFunction=drone.simple_goto):
    
    currentLocation = drone.location.global_relative_frame
    targetLocation = LocationGlobalRelative(dNorth, dEast,20)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while drone.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % drone.mode.name
        remainingDistance=get_distance_metres(drone.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.1: #Just below target, in case of undershoot.
            print("Reached target")
            break
        time.sleep(2)

def land():


    drone.mode = VehicleMode("LAND")  
    while not drone.location.global_relative_frame.alt <= 0.1:
        print(f"Altitude: {drone.location.global_relative_frame.alt} meters")
        time.sleep(1)
    print("Drone has landed.")
    drone.close()
    


mission = []

mission.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

## circle command 
mission.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,  -35.36265574 ,149.16512720 ,20))
mission.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, -35.36085911765,149.16512720 ,20))
mission.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, -35.36085911765,149.16292425 ,20))
mission.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, -35.36265574,149.16292425 ,20))
mission.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,  -35.36265574 ,149.16512720 ,20))

## control loop
mission.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,  -35.36265574 ,149.16512720 ,20))

##  infinite circuit
mission.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_JUMP,0, 0, 1, -1, 0, 0, 0, 0, 0))

take_off(10)

# Upload the mission
drone.commands.clear()
for cmd in mission:
    drone.commands.add(cmd)
drone.commands.upload()


while drone.mode != VehicleMode("AUTO"):
    drone.mode = VehicleMode("AUTO")
    time.sleep(0.5)

while not drone.commands.next == len(mission) - 2:
    print(f" Waypoint  {drone.commands.next}")
    time.sleep(1)

    while drone.commands.next == 6 :
                print(" 1 circle is finished. ")
                break

drone.commands.clear()


while drone.mode != VehicleMode("GUIDED"):
    drone.mode = VehicleMode("GUIDED")
    time.sleep(0.5)

goto(random_lat, random_lon)
goto(nearest_corner_lat, nearest_corner_lon)


while drone.mode != VehicleMode("AUTO"):
    drone.mode = VehicleMode("AUTO")
    time.sleep(0.5)
    
drone.commands.clear()
cmds = drone.commands
cmds.download()
cmds.wait_ready()

drone.commands.next = flag + 1
drone.commands.upload()

while True:
     
     if drone.commands.next == 6:
         print(" Cut the loop")
         break
     

while drone.mode != VehicleMode("GUIDED"):
    drone.mode = VehicleMode("GUIDED")
    time.sleep(0.5)

goto(random_lat_l,random_lon_l)
land()
drone.close()































# drone.commands.add(mission[8])
# drone.commands.add(mission[9])
# drone.commands.upload()

# drone.commands.next = 1

# while drone.commands.next == 1:

#     if distance_to_current_waypoint() <= 5:
#             time.sleep(1)
#             print("Reached the random point.")
#             drone.commands.next == 2

# while drone.commands.next == 2:

#     if distance_to_current_waypoint() <= 5:
#             time.sleep(1)
#             print("Reached the corner point.")

#             while drone.mode != VehicleMode("AUTO"):
#                 drone.mode = VehicleMode("AUTO")
#                 time.sleep(0.5)

#             drone.commands.clear()
#             drone.commands.next = flag + 1 
#             drone.commands.upload()

#             time.sleep(1)
#             break
        
                


    # drone.commands.clear()
    # cmd4=Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, random_lat_l, random_lon_l,20)
    # drone.commands.add(cmd4)
    # drone.commands.upload()