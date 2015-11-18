#! /usr/bin/env python2.7

import roslib
#roslib.load_manifest('graphs')
import sys
import inspect, os
from optparse import OptionParser
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv
import cv2
import numpy as np

#from qsr.msg import Objects

global x,y,z,rgb_flag,OBJ
global points
points = {'TL':{},'TR':{},'BR':{},'BL':{}}
points['TL']['x']=[]
points['TL']['y']=[]

points['TR']['x']=[]
points['TR']['y']=[]

points['BR']['x']=[]
points['BR']['y']=[]

points['BL']['x']=[]
points['BL']['y']=[]


def nothing(x):
    pass



#--------------------------------------------------------------------------------------#
def detect_and_draw(imgmsg):

    global x,y,rgb_flag,OBJ,points

    img = br.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
	
 #--------------------------------------- BLOB DETECTION ----------------------------------------------#
    # for detecting robot blocks
    # Blur each frame
    simg = cv2.GaussianBlur(img,(5,5),0)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(simg, cv2.COLOR_BGR2HSV)
    i = 2
    #for i in range(1,len(self.objects)+1):
    mask = cv2.inRange(hsv, np.array(OBJ[i][0:3]), np.array(OBJ[i][3:6]))
    # filter and fill the mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(OBJ[i][6],OBJ[i][6]))
    mask2 = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)

    ret,thresh = cv2.threshold(mask2,127,255,0)
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    my_contours = []

    centers = []
    for cnt in contours:
        cx = 0
        cy = 0
        if cv2.contourArea(cnt)>1400:
                my_contours.append(cnt)
                x,y,w,h = cv2.boundingRect(cnt)
                cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
                cx = x + (w/2) 
                cy = y + (h/2)    
                centers.append((cx,cy))
        
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
    if len(centers) == 4:
        points['TR']['x'].append(TR[0])
        points['TR']['y'].append(TR[1])
        points['TL']['x'].append(TL[0])
        points['TL']['y'].append(TL[1])
        points['BR']['x'].append(BR[0])
        points['BR']['y'].append(BR[1])
        points['BL']['x'].append(BL[0])
        points['BL']['y'].append(BL[1])
        print len(points['TR']['x'])
        avgs = []
        if len(points['TR']['x']) > 20:
            for key in points:
                
                x = np.mean(points[key]['x'])
                y = np.mean(points[key]['y'])
                avgs.append((x,y))
               
        print avgs
    cv2.drawContours(img,my_contours,-1,(0,255,0),3)

    cv2.imshow('RGB',img)
    cv2.setMouseCallback('RGB',print_coords)
    k = cv2.waitKey(5) & 0xFF
         
def print_coords(event,x,y,flags,param):
    #global ix,iy
    if event == cv2.EVENT_LBUTTONDBLCLK:
        print (x,y)

#--------------------------------------------------------------------------------------#

if __name__ == '__main__':	

    rgb_flag = 0

    br = CvBridge()

    directory = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    Objects_pointer = open(directory+'/objects.txt', 'r')
    OBJ = {}
    for line in Objects_pointer:
        line = line.strip(',\n')
        if line == 'END':
            break
        fields = line.split(',')
        fields = map(int, fields)
	OBJ[fields[0]] = fields[1:len(fields)]

    x = np.zeros(shape=(len(OBJ)), dtype=int)
    y = np.zeros(shape=(len(OBJ)), dtype=int)
    z = np.zeros(shape=(len(OBJ)), dtype=float)

    rospy.init_node('rosColordetect')
    image_topic = rospy.resolve_name("/cameras/left_hand_camera/image")      

    rospy.Subscriber(image_topic, sensor_msgs.msg.Image, detect_and_draw)


    rospy.spin()


