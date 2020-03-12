#!/usr/bin/env python

import rospy
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
bridge = CvBridge()

def image_callback(ros_image):
  print 'got an image'
  global bridge
  #convert ros_image into an opencv-compatible image
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
      print(e)
  #from now on, you can work exactly like with opencv
  small_frame = cv2.resize(cv_image, (0, 0), fx=0.5, fy=0.5)
  crop_img = small_frame
  hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

  lower_yellow = (10,10,0)
  upper_yellow = (10,10,10)

    # Threshold the HSV image to get only yellow colors
  mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
  res = cv2.bitwise_and(crop_img, crop_img, mask=mask)
  cv2.imshow("Image window2", res)
  cv2.imshow("Image window", hsv)
  cv2.waitKey(3)

  
def main(args):
  rospy.init_node('image_converter', anonymous=True)
  image_sub = rospy.Subscriber("/raspicam_node/image_raw",Image, image_callback)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)