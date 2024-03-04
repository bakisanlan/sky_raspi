
from dronekit import connect, VehicleMode
import time
import random
import math
from denekclass import DroneController



print("Trying to connect to the vehicle...")


drone = connect("127.0.0.1:14550", wait_ready=True)
print("Connected to the vehicle.")

# Drone kontrol sınıfı oluştur
drone_controller = DroneController(drone)

# Görevleri gerçekleştir
min_lat = -35.36265574
max_lat = -35.36085911765
min_lon = 149.16292425
max_lon = 149.16512720

# Görevleri gerçekleştir
drone_controller.take_off(40)
drone_controller.generate_mission()


# Otomatik modu aktifleştir
while drone.mode != VehicleMode("AUTO"):
        drone.mode = VehicleMode("AUTO")
        time.sleep(0.5)
# Görev tamamlanana kadar bekleyin
# while not drone_controller.cmds.next == len(drone_controller.mission) - 2:
#     time.sleep(1)

# Drone'u kapat
drone_controller.drone.close()




















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
