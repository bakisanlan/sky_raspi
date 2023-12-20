import os
import cv2
import numpy as np
import time
from threading import Thread
import importlib.util

class camera_class():
    """Camera object that controls video streaming from the Picamera"""
    def __init__(self, resolution=(640, 480), framerate=30, record_video=False):
        # Initialize the PiCamera and the camera image stream
        self.stream = cv2.VideoCapture(0)
        ret = self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        ret = self.stream.set(3, resolution[0])
        ret = self.stream.set(4, resolution[1])

        # Read the first frame from the stream
        (self.grabbed, self.frame) = self.stream.read()

        # Variable to control when the camera is stopped
        self.stopped = False

        # Video recording parameters
        self.record_video = record_video
        self.out = None
        self.recording = False
        self.start_time = time.time()
        self.video_duration = 30
        self.video_counter = 1

    def start(self):
        # Start the thread that reads frames from the video stream
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # Keep looping indefinitely until the thread is stopped
        while True:
            # If the camera is stopped, stop the thread
            if self.stopped:
                # Close camera resources
                self.stream.release()
                if self.record_video:
                    self.stop_recording()
                return

            # Otherwise, grab the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()

            # Record video if enabled
            if self.record_video and self.recording:
                self.out.write(self.frame)

                # Check if it's time to start a new video segment
                elapsed_time = time.time() - self.start_time
                if elapsed_time > self.video_duration:
                    self.stop_recording()
                    self.start_recording('output_{}.mp4'.format(self.video_counter), 30, (imW, imH))
                    self.video_counter += 1

    def read(self):
        # Return the most recent frame
        return self.frame

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
        self.stream.release()
        # Release the VideoWriter
        self.stop_recording()

    def detect_objects_with_coordinates(self, frame, boxes, classes, scores, labels, imW, imH):
        detected_objects = []

        for i in range(len(scores)):
            if 0.3 < scores[i] <= 1.0:
                ymin = int(max(1, (boxes[i][0] * imH)))
                xmin = int(max(1, (boxes[i][1] * imW)))
                ymax = int(min(imH, (boxes[i][2] * imH)))
                xmax = int(min(imW, (boxes[i][3] * imW)))

                center_x = int((xmin + xmax) / 2)
                center_y = int((ymin + ymax) / 2)

                object_name = labels[int(classes[i])]
                confidence = int(scores[i] * 100)

                detected_objects.append({
                    'object_name': object_name,
                    'confidence': confidence,
                    'center_x': center_x,
                    'center_y': center_y
                })

                # Draw bounding box
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (10, 255, 0), 2)

                # Draw label
                label = f'{object_name}: {confidence}%'
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                label_ymin = max(ymin, labelSize[1] + 10)
                cv2.rectangle(frame, (xmin, label_ymin - labelSize[1] - 10), (xmin + labelSize[0], label_ymin + baseLine - 10),
                            (255, 255, 255), cv2.FILLED)
                cv2.putText(frame, label, (xmin, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

                # Draw object coordinates
                coordinates_text = f'X: {center_x}, Y: {center_y}'
                cv2.putText(frame, coordinates_text, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        return detected_objects


# Define and set input arguments
MODEL_NAME = "custom_model_lite"
GRAPH_NAME = 'detect.tflite'  # Set the name of the .tflite file
LABELMAP_NAME = 'labelmap.txt'  # Set the name of the labelmap file
min_conf_threshold = 0.3
resW, resH = '1280x720'.split('x')
imW, imH = int(resW), int(resH)

# Import TensorFlow libraries
# If tflite_runtime is installed, import interpreter from tflite_runtime, else import from regular tensorflow
pkg = importlib.util.find_spec('tflite_runtime')
if pkg:
    from tflite_runtime.interpreter import Interpreter
else:
    from tensorflow.lite.python.interpreter import Interpreter



# Get the path to the current working directory
CWD_PATH = os.getcwd()

# Path to .tflite file, which contains the model that is used for object detection
PATH_TO_CKPT = os.path.join(CWD_PATH, MODEL_NAME, GRAPH_NAME)

# Path to label map file
PATH_TO_LABELS = os.path.join(CWD_PATH, MODEL_NAME, LABELMAP_NAME)

# Load the label map
with open(PATH_TO_LABELS, 'r') as f:
    labels = [line.strip() for line in f.readlines()]

# Have to do a weird fix for label map if using the COCO "starter model" from
# https://www.tensorflow.org/lite/models/object_detection/overview
# The first label is '???', which has to be removed.
if labels[0] == '???':
    del labels[0]

# Load the TensorFlow Lite model.
interpreter = Interpreter(model_path=PATH_TO_CKPT)

interpreter.allocate_tensors()

# Get model details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]

floating_model = (input_details[0]['dtype'] == np.float32)

input_mean = 127.5
input_std = 127.5

# Check the output layer name to determine if this model was created with TF2 or TF1,
# because outputs are ordered differently for TF2 and TF1 models
outname = output_details[0]['name']

if 'StatefulPartitionedCall' in outname:  # This is a TF2 model
    boxes_idx, classes_idx, scores_idx = 1, 3, 0
else:  # This is a TF1 model
    boxes_idx, classes_idx, scores_idx = 0, 1, 2

# Initialize frame rate calculation
frame_rate_calc = 1
freq = cv2.getTickFrequency()

# Initialize video stream
videostream = camera_class(resolution=(imW, imH), framerate=30, record_video=True).start()
videostream.start_recording('output_{}.mp4'.format(videostream.video_counter), 30, (imW, imH))
time.sleep(1)

while True:
    # Start timer (for calculating frame rate)
    t1 = cv2.getTickCount()

    # Grab frame from the video stream
    frame1 = videostream.read()

    # Acquire frame and resize to the expected shape [1xHxWx3]
    frame = frame1.copy()
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_resized = cv2.resize(frame_rgb, (width, height))
    input_data = np.expand_dims(frame_resized, axis=0)

    # Normalize pixel values if using a floating model (i.e., if the model is non-quantized)
    if floating_model:
        input_data = (np.float32(input_data) - input_mean) / input_std

    # Perform the actual detection by running the model with the image as input
    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()

    # Retrieve detection results
    boxes = interpreter.get_tensor(output_details[boxes_idx]['index'])[0]  # Bounding box coordinates of detected objects
    classes = interpreter.get_tensor(output_details[classes_idx]['index'])[0]  # Class index of detected objects
    scores = interpreter.get_tensor(output_details[scores_idx]['index'])[0]  # Confidence of detected objects

    # Call the function to detect objects and get their coordinates
    detected_objects = videostream.detect_objects_with_coordinates(frame, boxes, classes, scores, labels, imW, imH)
    for obj in detected_objects:
        object_name = obj['object_name']
        confidence = obj['confidence']
        center_x = obj['center_x']
        center_y = obj['center_y']

        # Use the extracted values as needed
        print(f"{object_name} at ({center_x}, {center_y}) with confidence {confidence}%")


    # Draw framerate in the corner of the frame
    cv2.putText(frame, 'FPS: {0:.2f}'.format(frame_rate_calc), (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2,
                cv2.LINE_AA)
    # All the results have been drawn on the frame, so it's time to display it
    cv2.imshow('Object detector', frame)

    # Calculate framerate
    t2 = cv2.getTickCount()
    time1 = (t2 - t1) / freq
    frame_rate_calc = 1 / time1

    # Press 'q' to quit
    if cv2.waitKey(1) == ord('q'):
        break

# Clean up
cv2.destroyAllWindows()
videostream.stop()
