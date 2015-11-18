#! /usr/bin/env python2.7

import roslib
from optparse import OptionParser
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2
import numpy as np
import visualization_msgs.msg
from geometry_msgs.msg import Pose,Point
import colorsys



import numpy as np
import cv2

global x,y
x={}
y={}

if __name__ == '__main__':

    br = CvBridge() # Create a black image, a window

#--------------------------------------------------------------------------------------#
    def detect_and_draw(imgmsg):
		global x,y

		img = br.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
		img = img[:,:,0:3]
		fgmask = fgbg.apply(img)

		cv2.imshow('rgb',img)
		#cv2.imshow('frame',fgmask)
		img1_bg = cv2.bitwise_and(img,img,mask = fgmask)
		img_obj = img.copy()
		for i in range(len(x)):
			print i
			img_obj[y[i],x[i],:] = [255,0,0]
		cv2.imshow('object',img_obj)
		cv2.imshow('masked',img1_bg)
		k = cv2.waitKey(1) & 0xff




#--------------------------------------------------------------------------------------#


    fgbg = cv2.BackgroundSubtractorMOG()
    rospy.init_node('object_detection')
    LH_image_topic = rospy.resolve_name("/cameras/left_hand_camera/image")

    rospy.Subscriber(LH_image_topic, sensor_msgs.msg.Image, detect_and_draw)

    rospy.spin()



