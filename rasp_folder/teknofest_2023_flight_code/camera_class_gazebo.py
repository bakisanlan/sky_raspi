import os
import cv2
import numpy as np
import time
from threading import Thread
import importlib.util
import roslib
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class camera_class():
    """Camera object that controls video streaming from the Picamera"""
    def __init__(self, resolution=(640, 480), record_video=False, showvideo=False):
        # Initialize the PiCamera and the camera image stream
        # creating image frame taker from gazebo env
        # '/webcam123/image_raw' is the address of camera in gazebo env
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/webcam123/image_raw",Image,self.gazeboCam,queue_size=10)


        # ret = self.video.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        # ret = self.video.set(3, resolution[0])
        # ret = self.video.set(4, resolution[1])

        # Read the first frame from the stream
        #(self.grabbed, self.frame) = self.video.read()

        # Initialize TensorFlow Lite model
        self.initialize_model()
        # Variable to control when the camera is stopped
        self.stopped = False

        # Video recording parameters
        self.record_video = record_video
        self.showvideo = showvideo
        resW, resH = '640x480'.split('x')
        self.imW, self.imH = int(resW), int(resH)

        self.out = None
        self.recording = False
        self.start_time = time.time()
        self.video_duration = 30
        self.video_counter = 1
        self.count = 0
        self.startProcess()


        if self.record_video:
            self.start_recording('output_{}.mp4'.format(self.video_counter), 30, (self.imW, self.imH))

    def gazeboCaminfo(self):
        print ("shutdown time!")

    def gazeboCam(self,data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def startProcess(self):
        # Start the thread that reads frames from the video stream
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # Keep looping indefinitely until the thread is stopped
        while True:
            # If the camera is stopped, stop the thread
            if self.stopped:
                # Close camera resources
                # self.video.release()
                if self.record_video:
                    self.stop_recording()
                return

            # Otherwise, grab the next frame from the stream
            # (self.grabbed, frame) = self.video.read()

            # Record video if enabled
            if self.record_video and self.recording:
                # Retrieve detection results
                    # Acquire frame and resize to the expected shape [1xHxWx3]
                frame_rgb = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
                frame_resized = cv2.resize(frame_rgb, (self.width, self.height))
                input_data = np.expand_dims(frame_resized, axis=0)
                input_mean = 127.5
                input_std = 127.5
                # Normalize pixel values if using a floating model (i.e., if the model is non-quantized)
                if self.floating_model:
                    input_data = (np.float32(input_data) - input_mean) / input_std

                # Perform the actual detection by running the model with the image as input
                self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
                self.interpreter.invoke()
                boxes = self.interpreter.get_tensor(self.output_details[self.boxes_idx]['index'])[0]  # Bounding box coordinates of detected objects
                scores = self.interpreter.get_tensor(self.output_details[self.scores_idx]['index'])[0]  # Confidence of detected objects
                self.detect_x_y(boxes, scores, self.imW, self.imH)

                # Write the frame with bounding boxes to the video file
                self.out.write(self.frame)

                # Check if it's time to start a new video segment
                elapsed_time = time.time() - self.start_time
                if elapsed_time > self.video_duration:
                    self.stop_recording()
                    self.start_recording('output_{}.mp4'.format(self.video_counter), 30, (self.imW, self.imH))
                    self.video_counter += 1
                    # All the results have been drawn on the frame, so it's time to display it
                if self.showvideo:
                    cv2.imshow('Object detector', self.frame)



    # def read(self):
    #     # Return the most recent frame
    #     return self.frame

    def start_recording(self, output_filename, fps, resolution):
        # Start recording video
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.out = cv2.VideoWriter(output_filename, fourcc, fps, resolution)
        self.recording = True
        self.start_time = time.time()

    def stop_recording(self):
        # Release the VideoWriter
        if self.out is not None:
            self.out.release()

    def stop(self):
        # Set the flag to stop the thread
        self.stopped = True
        # Release the video stream
        # self.video.release()
        # Release the VideoWriter
        self.stop_recording()


    def detect_x_y(self, boxes, scores, imW, imH):
        self.center_x= None
        self.center_y = None
        max_score_index =np.argmax(scores)
        if 0.3 < scores[max_score_index] <= 1.0:
            
            xmin = int(max(1, (boxes[max_score_index][1] * imW)))
            ymin = int(max(1, (boxes[max_score_index][0] * imH)))
            xmax = int(min(imW, (boxes[max_score_index][3] * imW)))
            ymax = int(min(imH, (boxes[max_score_index][2] * imH)))

            self.center_x = int((xmin + xmax) / 2)
            self.center_y = int((ymin + ymax) / 2)

            # Draw bounding box
            cv2.rectangle(self.frame, (xmin, ymin), (xmax, ymax), (10, 255, 0), 2)

            # Draw object coordinates
            coordinates_text = f'X: {self.center_x}, Y: {self.center_y}'
            cv2.putText(self.frame, coordinates_text, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)


    
    def initialize_model(self):
        # Define and set input arguments
        MODEL_NAME = "custom_model_lite"
        GRAPH_NAME = 'detect.tflite'
        LABELMAP_NAME = 'labelmap.txt'

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

        
    
# # Define and set input arguments
# MODEL_NAME = "custom_model_lite"
# GRAPH_NAME = 'detect.tflite'  # Set the name of the .tflite file
# resW, resH = '640x480'.split('x')
# imW, imH = int(resW), int(resH)

# # Import TensorFlow libraries
# # If tflite_runtime is installed, import interpreter from tflite_runtime, else import from regular tensorflow
# pkg = importlib.util.find_spec('tflite_runtime')
# if pkg:
#     from tflite_runtime.interpreter import Interpreter
# else:
#     from tensorflow.lite.python.interpreter import Interpreter


# # Get the path to the current working directory
# CWD_PATH = os.getcwd()

# # Path to .tflite file, which contains the model that is used for object detection
# PATH_TO_CKPT = os.path.join(CWD_PATH, MODEL_NAME, GRAPH_NAME)

# # Load the TensorFlow Lite model.
# interpreter = Interpreter(model_path=PATH_TO_CKPT)

# interpreter.allocate_tensors()

# # Get model details
# input_details = interpreter.get_input_details()
# output_details = interpreter.get_output_details()
# height = input_details[0]['shape'][1]
# width = input_details[0]['shape'][2]

# floating_model = (input_details[0]['dtype'] == np.float32)

# input_mean = 127.5
# input_std = 127.5

# # Check the output layer name to determine if this model was created with TF2 or TF1,
# # because outputs are ordered differently for TF2 and TF1 models
# outname = output_details[0]['name']

# if 'StatefulPartitionedCall' in outname:  # This is a TF2 model
#     boxes_idx, classes_idx, scores_idx = 1, 3, 0
# else:  # This is a TF1 model
#     boxes_idx, classes_idx, scores_idx = 0, 1, 2

# # Initialize frame rate calculation
# frame_rate_calc = 1
# freq = cv2.getTickFrequency()

# # Initialize video stream
# videostream = camera_class(resolution=(imW, imH), record_video=True, showvideo=True).start()
# time.sleep(1)

# while True:
#     # Start timer (for calculating frame rate)
#     t1 = cv2.getTickCount()

#     # Grab frame from the video stream
#     frame = videostream.read()

#     # Draw framerate in the corner of the frame
#     cv2.putText(frame, 'FPS: {0:.2f}'.format(frame_rate_calc), (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2,
#                 cv2.LINE_AA)



#     # Calculate framerate
#     t2 = cv2.getTickCount()
#     time1 = (t2 - t1) / freq
#     frame_rate_calc = 1 / time1

#     # Press 'q' to quit
#     if cv2.waitKey(1) == ord('q'):
#         videostream.stop()
#         break

# # Clean up
# cv2.destroyAllWindows()
# videostream.stop()

# ic = camera_class()
# rospy.init_node('camera_class', anonymous=True)
# rospy.spin()
