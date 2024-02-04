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

    def arm(self):
        while self.vehicle.is_armable is not True:
            print("UAV is not armable.")
            time.sleep(0.5)

        print("UAV can be armable.")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while self.vehicle.armed is not True:
            print("UAV is arming...")
            time.sleep(0.5)

        print("UAV is armed.")

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
        cmd_target = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 10, acc_rad, 0, 0, *target)
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
            time.sleep(0.2)
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
                self.def_mission.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 10, 0, 0, 0, *waypoint))

            # DO_JUMP
            self.def_mission.append(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_JUMP, 0, 0, 2, -1, 0, 0, 0, 0, 0))

            # Upload the mission
            for cmd in self.def_mission:
                self.vehicle.commands.add(cmd)
            self.vehicle.commands.upload()
            print("Commands are uploading.")
            self.count = 1

    def land(self):
        
        self.cmds.clear()
        time.sleep(0.2)
        self.cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 10, 0, 0, 0, 40.22948110, 29.00889869, 0))
        self.cmds.upload()
        time.sleep(0.2)
        print("Land commands added")

