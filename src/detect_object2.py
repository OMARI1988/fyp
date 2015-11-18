#! /usr/bin/env python2.7

import roslib
roslib.load_manifest('fyp')
import sys
import os
from optparse import OptionParser
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
#import cv
import cv2
import cv2.cv as cv
import numpy as np
#import scipy.ndimage.morphology as morphology
#from baxter_demos.msg import obj_relations,obj_hypotheses
import visualization_msgs.msg
from geometry_msgs.msg import Pose,Point
import colorsys
#from geometry_msgs.msg import Pose,Point


import numpy as np
import cv2

global x,y
x={}
y={}

if __name__ == '__main__':

    br = CvBridge()	# Create a black image, a window

#--------------------------------------------------------------------------------------#
    def detect_and_draw(imgmsg):
        global x,y
        img = br.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
        img = img[:,:,0:3]
        img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        img = cv2.medianBlur(img,5)
        #print img[img>0]
        cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

        circles = cv2.HoughCircles(img,cv.CV_HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=31,maxRadius=40)
        if circles == None:
            return
        circles = np.uint16(np.around(circles))
        centers = []
        for i in circles[0,:]:
            # draw the outer circle
            #print i[2]
            cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
            centers.append([i[0],i[1],i[2]])
        
        dis_TR = 100000
        dis_TL = 100000
        dis_BR = 100000
        dis_BL = 100000
        
        for i in centers:
            dis = np.sqrt((i[0]-1000)**2 + (i[1])**2)
            if dis < dis_TR:
                dis_TR = dis
                TR = i
            dis = np.sqrt((i[0])**2 + (i[1])**2)
            if dis < dis_TL:
                dis_TL = dis
                TL = i
            dis = np.sqrt((i[0]-1000)**2 + (i[1]-1000)**2)
            if dis < dis_BR:
                dis_BR = dis
                BR = i
            dis = np.sqrt((i[0])**2 + (i[1]-1000)**2)
            if dis < dis_BL:
                dis_BL = dis
                BL = i
                
        TR = map(int,TR)
        TL = map(int,TL)
        BR = map(int,BR)
        BL = map(int,BL)
        print img.shape
        #cv2.circle(cimg,(480,300),2,(0,0,0),3)
        # draw TR
        cv2.circle(cimg,(TR[0],TR[1]),TR[2],(0,255,0),2)
        cv2.circle(cimg,(TR[0],TR[1]),2,(0,0,255),3)
        cv2.circle(cimg,(TL[0],TL[1]),TL[2],(0,0,255),2)
        cv2.circle(cimg,(TL[0],TL[1]),2,(0,0,255),3)
        cv2.circle(cimg,(BR[0],BR[1]),BR[2],(255,0,0),2)
        cv2.circle(cimg,(BR[0],BR[1]),2,(0,0,255),3)
        cv2.circle(cimg,(BL[0],BL[1]),BL[2],(150,30,200),2)
        cv2.circle(cimg,(BL[0],BL[1]),2,(0,0,255),3)
        #print TR,TL,(TR[0]-TL[0])**2,TR[1]-TL[1]
        #dis = np.sqrt((TR[0]-TL[0])**2 + (TR[1]-TL[1])**2)
        #print dis/5
        dx = (float(TR[0])-float(TL[0]))/4
        dy = (float(TR[1])-float(TL[1]))/4
        dx2 = (float(BR[0])-float(BL[0]))/4
        dy2 = (float(BR[1])-float(BL[1]))/4

        MLx = TL[0]+int(np.round((float(BL[0])-float(TL[0]))/2))
        MLy = TL[1]+int(np.round((float(BL[1])-float(TL[1]))/2))
        MRx = TR[0]+int(np.round((float(BR[0])-float(TR[0]))/2))
        MRy = TR[1]+int(np.round((float(BR[1])-float(TR[1]))/2))
        
        dx3 = (MRx - MLx)/4
        dy3 = (MRy - MLy)/4
        
        cv2.circle(cimg,(MLx,MLy),2,(0,0,255),3)
        cv2.circle(cimg,(MRx,MRy),2,(0,0,255),3)
        
        for i in range(1,4):
            cv2.circle(cimg,(TL[0]+int(np.round(dx*i)),TL[1]+int(np.round(dy*i))),2,(0,0,255),3)
            print 'dx:',480-(TL[0]+int(np.round(dx*i))),'dy:',300-(TL[1]+int(np.round(dy*i)))
            print (TL[0]+int(np.round(dx*i)))
            
            cv2.circle(cimg,(BL[0]+int(np.round(dx2*i)),BL[1]+int(np.round(dy2*i))),2,(0,0,255),3)
            cv2.circle(cimg,(MLx+int(np.round(dx3*i)),MLy+int(np.round(dy3*i))),2,(0,0,255),3)
        print '--------------'
        #print dx,dy
        

        
        cv2.imshow('detected circles',cimg)
        cv2.waitKey(1)




#--------------------------------------------------------------------------------------#


    fgbg = cv2.BackgroundSubtractorMOG()
    rospy.init_node('object_detection')
    LH_image_topic = rospy.resolve_name("/cameras/left_hand_camera/image")

    rospy.Subscriber(LH_image_topic, sensor_msgs.msg.Image, detect_and_draw)

    rospy.spin()



