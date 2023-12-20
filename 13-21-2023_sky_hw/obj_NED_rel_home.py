import math
import numpy as np

def rotation_matrix(roll, pitch, yaw):
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    Rx = np.array([[1, 0, 0], [0, np.cos(roll), np.sin(roll)], [0, -np.sin(roll), np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, -np.sin(pitch)], [0, 1, 0], [np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

    return np.dot(np.dot(Rx, Ry), Rz)

def object_coordinates(euler_angles, pixel_position, camera_position_ned_rel_home, focal_length=620):
    roll, pitch, yaw = euler_angles
    x, y = -pixel_position[0], -pixel_position[1]

    R = rotation_matrix(roll, pitch + 27, yaw)

    north = (-camera_position_ned_rel_home[2]) * (R[0][0] * x + R[1][0] * y - focal_length * R[2][0]) / \
        (R[0][2] * x + R[1][2] * y - focal_length * R[2][2]) + camera_position_ned_rel_home[0]

    east = (-camera_position_ned_rel_home[2]) * (R[0][1] * x + R[1][1] * y - focal_length * R[2][1]) / \
        (R[0][2] * x + R[1][2] * y - focal_length * R[2][2]) + camera_position_ned_rel_home[1]

    return north, east, 0

# Örnek kullanım:
euler_angles = [10, 20, 30]  # Örnek Euler açıları (roll, pitch, yaw)
pixel_position = [100, 50]   # Örnek piksel konumu (x, y)
camera_position_ned_rel_home = [0, 0, 50]  # Örnek kamera konumu (NED koordinatları)

result = object_coordinates(euler_angles, pixel_position, camera_position_ned_rel_home)

print("Object Coordinates:", result)
