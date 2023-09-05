from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command

import time 
import math
print("Trying to connect to the vehicle...")
vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)
print("Connected to the vehicle.")

while True:
	
	print(math.degrees(vehicle.attitude.roll))
	print(math.degrees(vehicle.attitude.pitch))
	print(math.degrees(vehicle.attitude.yaw))
	print('--------------------------')
	
	time.sleep(0.5)
