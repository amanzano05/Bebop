#!/usr/bin/env python
#Importing all required libraries

from __future__ import print_function


import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  def __init__(self): #Is this a function to initiate?
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('bebop/image_raw',Image,self.callback)

  def callback(self,data):#Function to call the image back to bridge?
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)


def main(args):
  ic = image_converter() #Run image converter class
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt: #Kill the process if key pressed?
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':#Run main if name=main
    main(sys.argv)