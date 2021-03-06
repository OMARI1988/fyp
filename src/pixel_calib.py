#! /usr/bin/env python2.7

import roslib
#roslib.load_manifest('fyp')
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

global points
points = {'TL':{},'TM':{},'TR':{},'MR':{},'BR':{},'BM':{},'BL':{},'ML':{}}
points['TL']['x']=[]
points['TL']['y']=[]
points['TM']['x']=[]
points['TM']['y']=[]
points['TR']['x']=[]
points['TR']['y']=[]
points['MR']['x']=[]
points['MR']['y']=[]
points['BR']['x']=[]
points['BR']['y']=[]
points['BM']['x']=[]
points['BM']['y']=[]
points['BL']['x']=[]
points['BL']['y']=[]
points['ML']['x']=[]
points['ML']['y']=[]

if __name__ == '__main__':

    br = CvBridge()	# Create a black image, a window

#--------------------------------------------------------------------------------------#
    def detect_and_draw(imgmsg):
        
        def drawCross(x,y,size):
            mod = size * 10
            cv2.line(cimg,(x,y-mod),(x,y+mod),(255,50,100),size)
            cv2.line(cimg,(x-mod,y),(x+mod,y),(255,50,100),size)

        
        img = br.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
        img = img[:,:,0:3]
        img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        img = cv2.medianBlur(img,5)
        #print img[img>0]
        cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

        circles = cv2.HoughCircles(img,cv.CV_HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=10,maxRadius=40)
        if circles == None:
            return
        circles = np.uint16(np.around(circles))
        centers = []
        for i in circles[0,:]:

            centers.append([i[0],i[1],i[2]])
        
        dis_TR = 100000
        dis_TM = 100000
        dis_TL = 100000
        dis_ML = 100000
        dis_MR = 100000
        dis_BR = 100000
        dis_BM = 100000
        dis_BL = 100000
        
        for i in centers:
            dis = np.sqrt((i[0]-960)**2 + (i[1])**2)
            if dis < dis_TR:
                dis_TR = dis
                TR = i
            dis = np.sqrt((i[0])**2 + (i[1])**2)
            if dis < dis_TL:
                dis_TL = dis
                TL = i
            dis = np.sqrt((i[0]-960)**2 + (i[1]-600)**2)
            if dis < dis_BR:
                dis_BR = dis
                BR = i
            dis = np.sqrt((i[0])**2 + (i[1]-600)**2)
            if dis < dis_BL:
                dis_BL = dis
                BL = i
            dis = np.sqrt((i[0])**2 + (i[1]-300)**2)
            if dis < dis_ML:
                dis_ML = dis
                ML = i
            dis = np.sqrt((i[0]-960)**2 + (i[1]-300)**2)
            if dis < dis_MR:
                dis_MR = dis
                MR = i
            dis = np.sqrt((i[0]-480)**2 + (i[1])**2)
            if dis < dis_TM:
                dis_TM = dis
                TM = i
            dis = np.sqrt((i[0]-480)**2 + (i[1]-600)**2)
            if dis < dis_BM:
                dis_BM = dis
                BM = i
        if len(circles[0,:]) == 8:
            C = []
            global points
            points['TR']['x'].append(TR[0])
            points['TR']['y'].append(TR[1])
            points['TL']['x'].append(TL[0])
            points['TL']['y'].append(TL[1])
            points['BR']['x'].append(BR[0])
            points['BR']['y'].append(BR[1])
            points['BL']['x'].append(BL[0])
            points['BL']['y'].append(BL[1])
            points['ML']['x'].append(ML[0])
            points['ML']['y'].append(ML[1])
            points['MR']['x'].append(MR[0])
            points['MR']['y'].append(MR[1])
            points['TM']['x'].append(TM[0])
            points['TM']['y'].append(TM[1])
            points['BM']['x'].append(BM[0])
            points['BM']['y'].append(BM[1])
            print len(points['TR']['x'])
            if len(points['TR']['x']) > 20:
                for key in points:
                    
                    x = np.mean(points[key]['x'])
                    y = np.mean(points[key]['y'])
                    dis = np.sqrt((x-480)**2+(y-300)**2)
                    print key,'distance:',dis
                    
                    if key in ['TR','BR','TL','BL']:        
                        c = (133.0/383.0)/dis               # distance to the corners is 133mm 
                    else:                                       # distance to the table is 383mm
                        c = (94.0/383.0)/dis                # distance to sides is 94mm
                    print key,'c:',c
                    C.append(c)
                print 'final c:',np.mean(C)
                        
                     
                    
        TM = map(int,TM)
        TR = map(int,TR)
        TL = map(int,TL)
        BR = map(int,BR)
        BL = map(int,BL)
        BM = map(int,BM)
        ML = map(int,ML)
        MR = map(int,MR)

        drawCross(480,300,2)
        # draw TR
        cv2.circle(cimg,(TR[0],TR[1]),TR[2],(0,255,0),2)
        cv2.circle(cimg,(TR[0],TR[1]),2,(0,0,255),3)
        cv2.circle(cimg,(TL[0],TL[1]),TL[2],(0,0,255),2)
        cv2.circle(cimg,(TL[0],TL[1]),2,(0,0,255),3)
        cv2.circle(cimg,(BR[0],BR[1]),BR[2],(255,0,0),2)
        cv2.circle(cimg,(BR[0],BR[1]),2,(0,0,255),3)
        cv2.circle(cimg,(BL[0],BL[1]),BL[2],(150,30,200),2)
        cv2.circle(cimg,(BL[0],BL[1]),2,(0,0,255),3)
        cv2.circle(cimg,(ML[0],ML[1]),ML[2],(150,200,30),2)
        cv2.circle(cimg,(ML[0],ML[1]),2,(0,0,255),3)
        cv2.circle(cimg,(MR[0],MR[1]),MR[2],(30,200,150),2)
        cv2.circle(cimg,(MR[0],MR[1]),2,(0,0,255),3)
        cv2.circle(cimg,(TM[0],TM[1]),TM[2],(200,0,150),2)
        cv2.circle(cimg,(TM[0],TM[1]),2,(0,0,255),3)
        cv2.circle(cimg,(BM[0],BM[1]),BM[2],(100,100,10),2)
        cv2.circle(cimg,(BM[0],BM[1]),2,(0,0,255),3)
        
        

        
        cv2.imshow('detected circles',cimg)
        cv2.waitKey(1)



       

#--------------------------------------------------------------------------------------#


    fgbg = cv2.BackgroundSubtractorMOG()
    rospy.init_node('object_detection')
    LH_image_topic = rospy.resolve_name("/cameras/left_hand_camera/image")

    rospy.Subscriber(LH_image_topic, sensor_msgs.msg.Image, detect_and_draw)

    rospy.spin()



