from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    # creating image frame giver to gazebo env
    self.image_pub = rospy.Publisher("/webcam123/image_python",Image,queue_size=1)

    # creating image frame taker from gazebo env
    # '/webcam123/image_raw' is the address of camera in gazebo env
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/webcam123/image_raw",Image,self.callback,queue_size=10)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

ic = image_converter()
rospy.init_node('image_converter', anonymous=True)
rospy.spin()

