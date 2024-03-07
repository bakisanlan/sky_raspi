import time
import math
from dronekit import VehicleMode, LocationGlobalRelative, Command, mavutil

class DroneController:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.targetDistance = 0
        self.remainingDistance = 0
        self.cmds = None
        self.def_mission = []
        self.count = 0
        self.home_location = LocationGlobalRelative( 40.22910667, 29.00926226,40)

    def take_off(self,altitude):

        while self.vehicle.is_armable is not True:
            print(" Drone is not armable.")
            time.sleep(1)

        print("Drone is armable.")

        self.vehicle.mode = VehicleMode("GUIDED")

        self.vehicle.armed = True

        while self.vehicle.armed is not True:
            print("Drone is arming.")
            time.sleep(1)

        print("Drone armed.")
        self.vehicle.simple_takeoff(altitude)

        while self.vehicle.location.global_relative_frame.alt >= altitude*0.8:
            print("Drone is rising. ")
            time.sleep(1)


    def get_distance_metres(self, aLocation1, aLocation2):
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

    def goto(self, target, gotoFunction=None):
        if gotoFunction is None:
            gotoFunction = self.vehicle.simple_goto

        #currentLocation = self.vehicle.location.global_relative_frame
        #targetLocation = LocationGlobalRelative(target[0], target[1], target[2])
        #self.targetDistance = self.get_distance_metres(currentLocation, targetLocation)
        gotoFunction(target)
    
   
    def goto_auto(self, target):
        #print(self.cmds.list)
        # self.cmds.download()
        # self.cmds.wait_ready()
        self.cmds.clear()
        time.sleep(0.2)
        #print(self.cmds.list)
        acc_rad = 0
        target = [target.lat, target.lon, target.alt]
        cmd_target = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, acc_rad, 0, 0, *target)
        self.cmds.add(cmd_target)
        #self.cmds.next = 1
        self.cmds.upload()
        time.sleep(0.2)
        #print(self.cmds.list)
        # for a in self.cmds:
        #     print(a)


    def add_default_mission(self, waypoints=0, takeoff_alt=0):

        if self.count != 0:
            #print(self.cmds)
            # self.cmds.download()
            # self.cmds.wait_ready()
            self.cmds.clear()
            time.sleep(0.2)
            #print(self.cmds)
            for cmd in self.def_mission:
                self.cmds.add(cmd)
            self.cmds.upload()
           
            time.sleep(0.5)
            print("yüklenen görev command ",self.cmds.next)
            # for a in self.cmds:
            #     print(a)            
        else:
            self.cmds = self.vehicle.commands
            print('deneme')
            #self.def_mission = []
            self.waypoints = waypoints
            self.takeoff_alt = takeoff_alt

            self.cmds.download()
            self.cmds.wait_ready()
            self.vehicle.commands.clear()
            time.sleep(0.5)

            # TAKEOFF
            self.def_mission.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, takeoff_alt))

            for waypoint in waypoints:
                self.def_mission.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, *waypoint))

            # DO_JUMP
            self.def_mission.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_JUMP, 0, 0, 2, -1, 0, 0, 0, 0, 0))

            # Upload the mission
            for cmd in self.def_mission:
                self.vehicle.commands.add(cmd)
            self.vehicle.commands.upload()
            print("Commands are uploading.")
            self.count = 1

    def land(self):

        
        while True:

            print("land in içinde")
            time.sleep(1)
            while self.get_distance_metres(self.vehicle.location.global_relative_frame , self.home_location) <=5:
            
                self.cmds.clear()
                time.sleep(0.2)
            
                self.cmds.next = 0
                self.vehicle.commands.upload()


                        
                while self.vehicle.mode != VehicleMode("LAND"):
                    self.vehicle.mode = VehicleMode("LAND")
                    time.sleep(0.5)
                
                time.sleep(0.1)
                while not self.vehicle.location.global_relative_frame.alt <= 0.2:
                    print(f"Altitude: {self.vehicle.location.global_relative_frame.alt} meters")
                    time.sleep(1)
                print("Drone has landed.")
                self.vehicle.close()
        

