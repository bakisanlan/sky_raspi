import os
from datetime import datetime
import time
import math
import geopy.distance
import numpy as np
from camera_class import camera_class
from obj_NED_rel_home import obj_NED_rel_home
from geodetic_to_NED import geodetic_to_NED
from NED_to_lat_lon import NED_to_lat_lon
import importlib.util
from dronekit import connect


class Utility:
    def __init__(self,vehicle):
        # Connect to the vehicle
        self.vehicle = vehicle
        
        # Set home location
        self.home_location = self.vehicle.location.global_frame

        # Initialize TensorFlow Lite model
        self.initialize_model()

        # Initialize camera
        self.initialize_camera()

    def initialize_model(self):
        # Define and set input arguments
        MODEL_NAME = "custom_model_lite"
        GRAPH_NAME = 'detect.tflite'
        LABELMAP_NAME = 'labelmap.txt'
        resW, resH = '1280x720'.split('x')
        imW, imH = int(resW), int(resH)

        # Import TensorFlow libraries
        pkg = importlib.util.find_spec('tflite_runtime')
        if pkg:
            from tflite_runtime.interpreter import Interpreter
        else:
            from tensorflow.lite.python.interpreter import Interpreter

        # Get the path to the current working directory
        CWD_PATH = os.getcwd()

        # Path to .tflite file and label map file
        PATH_TO_CKPT = os.path.join(CWD_PATH, MODEL_NAME, GRAPH_NAME)
        PATH_TO_LABELS = os.path.join(CWD_PATH, MODEL_NAME, LABELMAP_NAME)

        # Load the label map
        with open(PATH_TO_LABELS, 'r') as f:
            self.labels = [line.strip() for line in f.readlines()]

        if self.labels[0] == '???':
            del self.labels[0]

        # Load the TensorFlow Lite model
        self.interpreter = Interpreter(model_path=PATH_TO_CKPT)
        self.interpreter.allocate_tensors()

        # Get model details
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]

        self.floating_model = (self.input_details[0]['dtype'] == np.float32)

        outname = self.output_details[0]['name']
        if 'StatefulPartitionedCall' in outname:
            self.boxes_idx, self.classes_idx, self.scores_idx = 1, 3, 0
        else:
            self.boxes_idx, self.classes_idx, self.scores_idx = 0, 1, 2
        # Perform object detection
        boxes = self.interpreter.get_tensor(self.output_details[self.boxes_idx]['index'])[0]
        scores = self.interpreter.get_tensor(self.output_details[self.scores_idx]['index'])[0]

    def initialize_camera(self):
        # Initialize camera
        self.camera_bot = camera_class(resolution=(imW, imH), framerate=30, record_video=True).start()
        resW, resH = '640x480'.split('x')
        imW, imH = int(resW), int(resH)


    def print_obj_loc(self, detected_obj_px):
        if detected_obj_px != None:
            vehicle_location = self.vehicle.location.global_frame
            #vehicle_ned_rel_home = geodetic_to_NED((vehicle_location.alt,vehicle_location.lon, vehicle_location.lat),(home_location.lat,home_location.lon, home_location.alt))
            #vehicle_ned_rel_home = geodetic_to_NED((vehicle_location.alt,vehicle_location.lon, vehicle_location.lat),(40,20,0))
            vehicle_ned_rel_home = geodetic_to_NED(self.home_location,vehicle_location)
            obj_ned_rel_home = obj_NED_rel_home([math.degrees(self.vehicle.attitude.roll),math.degrees(self.vehicle.attitude.pitch),math.degrees(self.vehicle.attitude.yaw)],detected_obj_px,list(vehicle_ned_rel_home))
            obj_ned_rel_vehicle = [obj_ned_rel_home[0]-vehicle_ned_rel_home[0], obj_ned_rel_home[1]-vehicle_ned_rel_home[1], obj_ned_rel_home[2]-vehicle_ned_rel_home[2]]
            #now = datetime.datetime.now()
            #print(obj_ned_rel_home)
            #print('girdim')
            time.sleep(0.2)
            #print('Home location {}'.format())            
            print('Vehicle NED location is {} /n '.format(vehicle_ned_rel_home))
            print('Object NED location is {} /n'.format(obj_ned_rel_home))
            print('Guessed Obj Location is = {} north {} east /n'.format(obj_ned_rel_vehicle[0],obj_ned_rel_vehicle[1]))
            print('roll: {} pitch: {} yaw{} /n'.format(math.degrees(self.vehicle.attitude.roll),math.degrees(self.vehicle.attitude.pitch),math.degrees(self.vehicle.attitude.yaw)))

    def obj_px_to_obj_lat_lon(self, detected_obj_px):
            vehicle_location = self.vehicle.location.global_frame
            #vehicle_ned_rel_home = geodetic_to_NED((vehicle_location.alt,vehicle_location.lon, vehicle_location.lat),(home_location.lat,home_location.lon, home_location.alt))
            #vehicle_ned_rel_home = geodetic_to_NED((vehicle_location.alt,vehicle_location.lon, vehicle_location.lat),(40,20,0))
            vehicle_ned_rel_home = geodetic_to_NED(self.home_location,vehicle_location)
            obj_ned_rel_home = obj_NED_rel_home([math.degrees(self.vehicle.attitude.roll),math.degrees(self.vehicle.attitude.pitch),math.degrees(self.vehicle.attitude.yaw)],detected_obj_px,list(vehicle_ned_rel_home))
            obj_ned_rel_vehicle = [obj_ned_rel_home[0]-vehicle_ned_rel_home[0], obj_ned_rel_home[1]-vehicle_ned_rel_home[1], obj_ned_rel_home[2]-vehicle_ned_rel_home[2]]
            return NED_to_lat_lon(vehicle_location, obj_ned_rel_vehicle[0],obj_ned_rel_vehicle[1])

    def get_obj_mean_lat_lon(self, detected_obj_px):
            #list_detected_obj_px = [detected_obj_px]
            list_obj_lat_lon = [self.obj_px_to_obj_lat_lon(detected_obj_px)]
            while True:
                time_start = time.time()
                while (time.time() - time_start) < 0.2: 
                    detected_obj_px = self.camera_bot.detect_x_y(self.boxes, self.scores, self.imW, self.imH)
                    while detected_obj_px != None:
                        list_obj_lat_lon.append(self.obj_px_to_obj_lat_lon(detected_obj_px))
                        detected_obj_px = self.camera_bot.detect_x_y(self.boxes, self.scores, self.imW, self.imH)
                        time_start = time.time()    
                break
            if len(list_obj_lat_lon) == 1:  # == 1 added 
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
                zipped_data = zip(lat_values,lon_values)
                filtered_data = [(x, y) for (x, y), x_z, y_z in zip(zipped_data, lat_z_scores, lon_z_scores) if abs(x_z) <= threshold and abs(y_z) <= threshold]
                filtered_lat_values = [lat for lat, lon in filtered_data]
                filtered_lon_values = [lon for lat, lon in filtered_data]
                filtered_obj_lat_mean = np.mean(filtered_lat_values)
                filtered_obj_lon_mean = np.mean(filtered_lon_values)
                return (filtered_obj_lat_mean, filtered_obj_lon_mean)

    def get_obj_mean_lat_lon_wp(self, detected_obj_px,obj_mean_lat_lon):
            #list_detected_obj_px = [detected_obj_px]
            list_obj_lat_lon = [self.obj_px_to_obj_lat_lon(detected_obj_px)]
            while True:
                time_start = time.time()
                while (time.time() - time_start) < 0.2:
                    detected_obj_px = self.camera_bot.detect_x_y(self.boxes, self.scores, self.imW, self.imH)
                    while (detected_obj_px != None):
                        #obj_mean_lat_lon = self.get_obj_mean_lat_lon(detected_obj_px)   # added 
                        dist_vehicle_obj = self.distance_fun(obj_mean_lat_lon,self.vehicle.location.global_frame)
                        if dist_vehicle_obj > 10:
                            list_obj_lat_lon.append(self.obj_px_to_obj_lat_lon(detected_obj_px))
                            detected_obj_px = self.camera_bot.detect_x_y(self.boxes, self.scores, self.imW, self.imH)
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
                zipped_data = zip(lat_values,lon_values)
                filtered_data = [(x, y) for (x, y), x_z, y_z in zip(zipped_data, lat_z_scores, lon_z_scores) if abs(x_z) <= threshold and abs(y_z) <= threshold]
                filtered_lat_values = [lat for lat, lon in filtered_data]
                filtered_lon_values = [lon for lat, lon in filtered_data]
                filtered_obj_lat_mean = np.mean(filtered_lat_values)
                filtered_obj_lon_mean = np.mean(filtered_lon_values)
                return (filtered_obj_lat_mean, filtered_obj_lon_mean)                
        
    def distance_fun(self, target, reference):
        distance = geopy.distance.GeodesicDistance((target[0],target[1]),
                                                (reference.lat,
                                                reference.lon)).meters
        return abs(distance)

    def print_now(self):
        current_time = datetime.now().strftime("%H:%M:%S")
        print(current_time)
