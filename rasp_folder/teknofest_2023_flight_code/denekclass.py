from dronekit import Command, VehicleMode, LocationGlobalRelative
import time
import random
import math
from pymavlink import mavutil

class DroneController:
    def __init__(self, drone):
        self.drone = drone
        self.flag = 1
        self.cmds = None
        self.mission = []

    def take_off(self, altitude):
        while not self.drone.is_armable:
            print("Drone is not armable.")
            time.sleep(1)

        print("Drone is armable.")
        self.drone.mode = VehicleMode("GUIDED")
        self.drone.armed = True

        while not self.drone.armed:
            print("Drone is arming.")
            time.sleep(1)

        print("Drone armed.")
        self.drone.simple_takeoff(altitude)

        while self.drone.location.global_relative_frame.alt >= altitude * 0.9:
            print("Drone is rising.")
            time.sleep(1)

    def haversine(self, lat1, lon1, lat2, lon2):
        R = 6371000  # Radius of the Earth in meters
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(
            dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c
        return distance

    def calculate_nearest_corner(self, random_lat, random_lon, min_lat, max_lat, min_lon, max_lon):
        corner1 = (min_lat, max_lon)
        corner2 = (max_lat, max_lon)
        corner3 = (max_lat, min_lon)
        corner4 = (min_lat, min_lon)

        corners = [corner1, corner2, corner3, corner4]

        shortest_distance = None
        nearest_corner = None

        for corner in corners:
            distance = self.haversine(random_lat, random_lon, corner[0], corner[1])
            if shortest_distance is None or distance < shortest_distance:
                shortest_distance = distance
                nearest_corner = corner

        for i in corners:
            if nearest_corner == i:
                break
            else:
                self.flag += 1

        if nearest_corner is not None:
            print("Nearest corner is:", nearest_corner)
            print("Shortest distance is:", shortest_distance)
            print("Nearest corner is: ", self.flag)

        return nearest_corner

    def get_distance_metres(self, aLocation1, aLocation2):
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

    def goto(self, dNorth, dEast, gotoFunction=None):
        currentLocation = self.drone.location.global_relative_frame
        targetLocation = LocationGlobalRelative(dNorth, dEast, 20)
        targetDistance = self.get_distance_metres(currentLocation, targetLocation)

        if gotoFunction:
            gotoFunction(targetLocation)

            while self.drone.mode.name == "GUIDED":
                remainingDistance = self.get_distance_metres(self.drone.location.global_relative_frame, targetLocation)
                print("Distance to target: ", remainingDistance)
                if remainingDistance <= targetDistance * 0.1:
                    print("Reached target")
                    break
                time.sleep(2)

    def land(self):
        self.drone.mode = VehicleMode("LAND")
        while not self.drone.location.global_relative_frame.alt <= 0.1:
            print(f"Altitude: {self.drone.location.global_relative_frame.alt} meters")
            time.sleep(1)
        print("Drone has landed.")

    def generate_mission(self):

        

        self.mission.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

        ## circle command 
        self.mission.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,  -35.36265574 ,149.16512720 ,20))
        self.mission.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, -35.36085911765,149.16512720 ,20))
        self.mission.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, -35.36085911765,149.16292425 ,20))
        self.mission.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, -35.36265574,149.16292425 ,20))
        self.mission.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,  -35.36265574 ,149.16512720 ,20))

        ## control loop
        self.mission.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,  -35.36265574 ,149.16512720 ,20))

        ##  infinite circuit
        self.mission.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_JUMP,0, 0, 1, -1, 0, 0, 0, 0, 0))


        # Add your mission commands here...

       
