#!/usr/bin/env python


#Importing all required libraries
from __future__ import print_function
#alfonso manzano

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

#callback library for subcriber for video
#the function recieves as data of tipy image as argument.
def camera_callback(data):
    #bridge is used as global object for the brigde
    global bridge
    #used to store the "image" resulting from the bridge
    stream=0
    #tries to get the "image from the bridge"
    #if succesed show the image 
    #the "video" resulted is composite of many images precessed for the bridge
    try:
        face_cascade = cv2.CascadeClassifier('lbpcascade_frontalface_improved.xml')
        stream=bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv2.cvtColor(stream, cv2.COLOR_BGR2GRAY)
    except CvBridgeError as e:
        print(e)
    faces = face_cascade.detectMultiScale(gray, 
                                      scaleFactor=1.2,
                                      minNeighbors=5,
                                      minSize=(50,50),
                                      flags = cv2.CASCADE_SCALE_IMAGE)
    for (x,y,w,h) in faces:
        stream = cv2.rectangle(stream,(x,y),(x+w,y+h),(255,0,0),2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = stream[y:y+h, x:x+w]
    cv2.imshow("Video", stream)
    cv2.waitKey(3)

##########main method#################
if __name__ == '__main__':
    #initialize the node 
    rospy.init_node('Video_Stream', anonymous=True)
    #global variables for objects
    global bridge
    global imageSub
    #creates the bridge
    bridge=CvBridge()
    #subscribe to the topic
    imageSub=rospy.Subscriber('bebop/image_raw', Image, camera_callback)
    #keeps ROS running
    rospy.spin()
