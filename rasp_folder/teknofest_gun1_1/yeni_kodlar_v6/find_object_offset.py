import numpy as np
import math

#origin of the camera axis px is width/2,height/2

def find_object_offset(euler_ang,obj_px_pos,camera_pos):

    roll = -(euler_ang[1] + 27)      # effect y axis(pitch) negative, #27 is preconfigure pitch angle of camera
    pitch = -euler_ang[0]     # effect x axis(roll)  pozitive
    yaw = euler_ang[2]      # effect both x and y axis positive

    # Define the camera's Euler angles
    roll = math.radians(roll)  # radians
    pitch = math.radians(pitch)  # radians
    yaw = math.radians(yaw)  # radians

    # Define the camera's intrinsic parameters
    camera_matrix = np.array([[0.0616, 0, 0],    #for 640,480px 
                            [0, 0.0616, 0],
                            [0, 0, 1]])

    # Define the camera's extrinsic parameters
    # Calculate the rotation matrix that transforms camera coordinates to world coordinates
    Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    R = np.dot(np.dot(Rz, Ry), Rx)

    # Define the image coordinates of the object
    image_coords = np.array([obj_px_pos[0], obj_px_pos[1], 1])  # pixels

    # Calculate the object's position in the camera frame
    object_coords_camera = np.transpose(camera_matrix).dot(image_coords)

    # Transform the object coordinates to world coordinates
    object_coords_world = np.dot(np.transpose(R), object_coords_camera - camera_pos)

    # Calculate the distance to the object
    distance = np.linalg.norm(object_coords_world)

    # print("Object coordinates in camera frame: ({:.2f}, {:.2f}, {:.2f}) meters".format(object_coords_camera[0],
    #                                                                                     object_coords_camera[1],
    #                                                                                     object_coords_camera[2]))
    # print("Object coordinates in world frame: ({:.2f}, {:.2f}, {:.2f}) meters".format(object_coords_world[0],
    #                                                                                     object_coords_world[1],
    #                                                                                     object_coords_world[2]))
    # print("Distance to object: {:.2f} meters".format(distance))

    return (object_coords_world[0],object_coords_world[1])

print(find_object_offset((0,15,0),(0,0),(0,0,50)))
