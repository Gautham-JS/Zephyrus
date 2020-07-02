#!/usr/bin/env python
from __future__ import print_function
import time 
#import roslib
#roslib.load_manifest('cvg_sim_gazebo')
import sys
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

rospy.init_node('image_converter', anonymous=True)
bridge = CvBridge()
cap = cv2.VideoCapture(0)
#cap = cv2.VideoCapture("/home/gautham/Videos/cam_video.mp4")
image_pub = rospy.Publisher("image_topic_2",Image)
vid_cod = cv2.VideoWriter_fourcc(*'XVID')
output = cv2.VideoWriter("cam_video.mp4", vid_cod, 30, (640,480))
while not rospy.is_shutdown():
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

    print("publishin")
    #time.sleep(0.07)
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
output.release()
cv2.destroyAllWindows()