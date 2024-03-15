# There are 5 different main frames that are image frame, camera frame,
# body frame of UAV, local NED frame of UAV and inertial NED frame

# Aim is that function is finding relative position of object,that 
# is detected in image frame, w.r.t UAV in local NED frame.

# For making that frame conversion we have to do it with step by step.
# Rotation and translation steps are --> 
# 1) Image frame to camera frame
# 2) Camera frame to body frame of UAV
# 3) Body frame to local NED frame of UAV

###############################################################################################
##### DETAILED INFORMATION OF IMPORTANT FRAMES #####

# Camera Frame = [Xc; Yc; Zc] --> Origin point of that frame is assumed to be in
# cg of UAV(not true in real scnerio). Z direction of Camera Frame is pointing 
# outside direction trough perpendicular to lens of camera. X direction
# of camera is pointing right wing direction. And Y direction of camera 
# is pointing nose direction of UAV. As you can see X and Y position is
# totally inverted according to X and Y directions of body frame of UAV.
# And also, Z direction of Camera Frame and Z direction of body frame of UAV
# are coincident.

# Image Frame = [Xp; Yp; focal length] --> Image Frame is a 2D frame that
# is translated by amount of focal length distance from Camera Frame origin pos.
# X and Y direction is same with Camera Frame. Due to Image Frame is 2D, Z position
# is constant as focal length.

# Body Frame = [Xb; Yb; Zb] --> Origin point of that frame is cg of UAV.
# X direction is pointing nose direction of UAV.
# Y direction is pointing right wing direction and Z direction is perpendicular 
# to those X and Y direction w.r.t right hand side rule.

# Local NED Frame = [Xlned; Ylned; Zlned] --> Origin point of that frame is cg of UAV.
# X direction is pointing true North and Y direction is pointing true East. Z direction
# is perpendicular to those X and Y direction w.r.t right hand side rule.

# Inertial NED Frame = [Xined; Yined; Zined] --> Origin point of that frame is home location 
# that is fixed in the specific point on world surface.
# X direction is pointing true North and Y direction is pointing true East. Z direction
# is perpendicular to those X and Y direction w.r.t right hand side rule.
    
###############################################################################################

import math
import numpy as np

def rotation_matrix(roll, pitch, yaw):
    
    # Rotation matrix definition with positive direction that is right hand side convention
    
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    Rx = np.array([[1, 0, 0], [0, np.cos(roll), np.sin(roll)], [0, -np.sin(roll), np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, -np.sin(pitch)], [0, 1, 0], [np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), np.sin(yaw), 0], [-np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

    return np.dot(np.dot(Rx, Ry), Rz)

def object_coordinates(euler_angles, pixel_position, cam_pos_ned_inertia, focal_length=620):
    
    roll, pitch, yaw = euler_angles
    
    Xp, Yp = pixel_position[0], pixel_position[1] 
    
    # Zc ~= h/(cos(pitch) * cos(roll)), this formulation provide find Zc in appoximately, still 
    # there is error due to projection of X-Y plane of Camera Frame
    h = abs(cam_pos_ned_inertia[2]) # Due to NED frame of home, we should take absolute value of Z for finding altitude
    Zc = h / (math.cos(math.radians(pitch)) * math.cos(math.radians(roll)))
    
    # k represent the rate of (Number of pixels/Meters In Real World). It is can be found by taking 
    # rate of focal length and Zc. For intiutive and detailed information https://www.youtube.com/watch?v=JxwLyZ-I7vI
    k = focal_length/Zc
    
    # Converting Image Frame to Camera Frame of object pos.
    # Xp/Xc = k,  Yp/Yc = k
    Xc = Xp/k
    Yc = Yp/k
    
    # As mentioned above explanation, body X-Y plane and camera X-Y plane is inverted and Z is coincident.
    # Converting Camera Frame to Body frame of object pos.
    Xb = Yc
    Yb = Xc
    Zb = Zc
    obj_pos_bodyF = np.array([[Xb], [Yb], [Zb]])
 
    # Rotation matrix from Local NED Frame to Body Frame
    R_lned2b = rotation_matrix(roll, pitch, yaw)
    
    # Rotation matrix from Body Frame to Local NED Frame
    R_b2lned = np.linalg.inv(R_lned2b)
    
    # Rotation from Body Frame to Local NED Frame
    obj_pos_localnedF = np.dot(R_b2lned, obj_pos_bodyF)
    
    # Return (N, E) relative to cg of UAV
    return (obj_pos_localnedF[0][0],obj_pos_localnedF[1][0])

# Example of use
euler_angles = [0, 0, 90]  # Euler Angles of UAV
pixel_position = [200, 100]   # Pixel position of object in image frame
cam_pos_ned_inertia = [0, 0, -50]  # Camera position in Inertia NED frame

result = object_coordinates(euler_angles, pixel_position, cam_pos_ned_inertia)

print("Object Coordinates Rel To UAV in (North-East)", result)
