#!/usr/bin/env python

import roslib
import sys
import inspect, os
import rospy
import string
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from moveit_commander import conversions
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
import baxter_interface
from cv_bridge import CvBridge
import cv
import cv2
import numpy as np
import math

import random
from BoardClass3D import Board


import rospy
import sys
import numpy as np
from ros_mary_tts.srv import *
import baxter_interface
from std_msgs.msg import (Empty,Bool)

# init rospy node
rospy.init_node('pickup_and_stack')

# file directory
directory = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

#------------------------------------------------------------------#
def speak(x):
    rospy.wait_for_service('ros_mary')
    try:
        add_two_ints = rospy.ServiceProxy('ros_mary',ros_mary)
        resp1 = add_two_ints(x)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def main():
    
    speak('Get ready to play.')
    # get setup parameters
    limb, distance = get_distance_data()

    # read colour data from file
    colours = get_object_data()

    # create board class
    theBoard = Board()

    # create stack class instance
    baxter = robot(limb, distance, colours, theBoard)    
    baxter.send_image('default')

    print "limb     = ", limb
    print "distance = ", distance


    # open the gripper
    baxter.gripper.open()

    # move to start position
    speak('Please wait while I get ready.')
    baxter.start_position()
    
    baxter.send_image('locateboard')
    # wait for camera feedback
    rospy.sleep(1)
    baxter.toggleDisplayFeed(1)
    
    speak('Locating the board.')
    baxter.locate_board()  
    baxter.send_image('default')
    baxter.start_position()  
    game = True                    
    while game:
        theBoard.printBoard()
        baxter.detect_pieces() 
        theBoard.printBoard()
        if theBoard.checkWinner(baxter.playerPiece):
            baxter.send_image('anger2')
            theBoard.printBoard()
            speak('Hooray! You have won the game!')
            print('Hooray! You have won the game!')
            break
        elif theBoard.isFull(): 
            baxter.send_image('default')
            theBoard.printBoard()
            speak('The game is a tie!')
            print('The game is a tie!')
            break
        baxter.send_image('default')    
        baxter.take_turn()
        
        if theBoard.checkWinner('R'):
            baxter.send_image('happy2')
            theBoard.printBoard()
            speak('Lucas won !, I enjoyed playing with you.')
            print('The computer has beaten you! You lose.')
            game = False
        elif theBoard.isFull(): 
            baxter.send_image('default')
            theBoard.printBoard()
            speak('The game is a tie!')
            print('The game is a tie!')
            break


    sys.exit()

    rospy.spin()

class robot():
    def __init__(self, arm, distance, colours, board):

        # piece data
        self.playerPiece = 'X'
        self.playerPieceIndex = 1    # ( 1 or 2, 3 = board)
        self.robotPiece = 'R'
        self.robotPieceIndex = 2
    
        # space detection
        self.board = board
        self.boardHeight = 0.2971
        self.boardWidth = 0.410
        self.storePointPix = True
        self.detectedAllCorners = False
        self.pixelPoints = []
        self.board_tolerance = 0.00375
        
        # board xyz
        spaces = self.board.getSpaces()
        self.boardZ = len(spaces)        # levels
        self.boardY = len(spaces[0])     # rows   
        self.boardX = len(spaces[0][0])  # columns
        
        # Background subtraction
        self.bgsubImgmsg = None 
        self.imgmsg = None
        
        # Circle piece detection
        self.numCircleFrames = 1
        self.spacePixelDist = 50.0
       
        # arm ("left" or "right")
        self.limb           = arm
        self.limb_interface = baxter_interface.Limb(self.limb)

        if arm == "left":
            self.other_limb = "right"
        else:
            self.other_limb = "left"

        self.other_limb_interface = baxter_interface.Limb(self.other_limb)

        # gripper ("left" or "right")
        self.gripper = baxter_interface.Gripper(arm)

        # object color dictionary data
        self.objects = colours

        # zeroed lists for pixel coordinates of objects        
        self.x = np.zeros(shape=(len(colours)), dtype=int)
        self.y = np.zeros(shape=(len(colours)), dtype=int)

        # speeds
        self.normal = 0.8
        self.slow = 0.1

        # start positions
        self.start_pos_x = 0.50                        # x     = front back
        self.start_pos_y = 0.30                        # y     = left right
        self.start_pos_z = 0.15                        # z     = up down
        self.roll        = -1.0 * math.pi              # roll  = horizontal
        self.pitch       = -0.0 * math.pi               # pitch = vertical
        self.yaw         = 0.0 * math.pi               # yaw   = rotation


        self.pose = [self.start_pos_x, self.start_pos_y, self.start_pos_z,     \
                     self.roll, self.pitch, self.yaw]

        # distances
        self.distance = distance 
        #self.pixelHeightConst = 0.0022762               # constant for distance = 0.383, change for different heights!!!   
        self.block_height = 0.04
        self.block_pickup_height = self.distance - self.block_height
        self.block_grip_height = 0.102
        self.block_tolerance = 0.005
       

        # camera parameters (NB. other parameters in open_camera)
       # self.cam_calib    = 0.0025                     # original   # meters per pixel at 1 meter
        self.cam_calib = 0.0022762                      # constant for distance = 0.383, change for different heights!!! 
        self.cam_x_offset = 0.045                       # original camera gripper offset
        self.cam_y_offset = -0.015 #-0.01
        #self.cam_x_offset = 0.045                      # camera gripper offset
        #self.cam_y_offset = -0.02
        self.width        = 960                        # Camera resolution
        self.height       = 600

        # Enable the actuators
        baxter_interface.RobotEnable().enable()

        # set speed as a ratio of maximum speed
        self.limb_interface.set_joint_position_speed(0.8)
        self.other_limb_interface.set_joint_position_speed(0.8)

        # calibrate the gripper
        self.gripper.calibrate()

        # set camera resolution
        self.config_camera(self.limb, self.width, self.height)

        # subscribe to required camera
        self.subscribe_to_camera(self.limb)
        
        # Display camera feed to face display
        self.displayCamera = False
        self.displayContours = False
        self.pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
        
        # move other arm out of harms way
        ## starting location of the robot
        if arm == "left":
            self.baxter_ik_move("right", (0.7, -0.40, -0.10, math.pi, 0.0, 0.0))
        else:
            self.baxter_ik_move("left", (0.25, 0.50, 0.2, math.pi, 0.0, 0.0))
    
    def toggleDisplayFeed(self,on,contours=False):
        if contours:
            self.displayContours = True
        else:
            self.displayContours = False
        if on:
            self.displayCamera = True
        else:
            self.displayCamera = False

    def send_image(self,img_name):
        """
        Send the image located at the specified path to the head
        display on Baxter.

        @param path: path to the image file to load and send
        """
        
        self.toggleDisplayFeed(0) # Turn off feed to head display
        
        global directory 


        path = directory+'/images/'+img_name+'.png'
        img = cv2.imread(path)
        #msg = CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        self.publish_img(img,False)
        rospy.sleep(1)
        # Sleep to allow for image to be published.


    def publish_img(self,img,resize=True):
        
        if resize:
            img = cv2.resize(img, (1024, 600)) 
        msg = CvBridge().cv2_to_imgmsg(img, encoding="passthrough")
        self.pub.publish(msg)

                                                        # Game Logic
#--------------------------------------------------------------------------------------------------------------------------------------------#

    def take_turn(self):
    

        def random_move(movesList,level):
            # Returns a valid move from the passed list on the passed board.
            # Returns None if there is no valid move.
            possibleMoves = []
            for i in movesList:
                if not self.board.getPiece(level,i[0],i[1]):
                        
                    possibleMoves.append(i)

            if len(possibleMoves) != 0:
                return random.choice(possibleMoves)
            else:
                return None

                        
        # check to see if winning move is possible
        for row in range(self.boardY):
            for space in range(self.boardX):
                copy = self.board.getCopy()
                if  not self.board.getPiece(self.boardZ-1,row,space): # if space is free 
                    copy.makeMove(row,space, self.robotPiece)
                    if copy.checkWinner(self.robotPiece):
                        z = self.board.makeMove(row,space, self.robotPiece)
                        self.place_piece(row, space, z)
                        return
                            
        # Check if the human player could win on his next move, and block them.
        for row in range(self.boardY):
            for space in range(self.boardX):
                copy = self.board.getCopy()
                if  not self.board.getPiece(self.boardZ-1,row,space): # if space is free 
                    copy.makeMove(row,space, self.playerPiece)
                    if copy.checkWinner(self.playerPiece):
                        z = self.board.makeMove(row,space, self.robotPiece)
                        self.place_piece(row, space, z)
                        return
  
         # Try to take the middle center, if it's free and has a robot piece below it.
        if not self.board.getPiece(1,1,1) and self.board.getPiece(0,1,1) == self.robotPiece:
            level = self.board.makeMove(1,1,self.robotPiece)
            self.place_piece(1,1,level)
            return            
                
        # Try to take the bottom center, if it's free.
        if not self.board.getPiece(0,1,1):
            level = self.board.makeMove(1,1,self.robotPiece)
            self.place_piece(1,1,level)
            return 
        
        # Try to take one of the corners of the bottom level, if they are free.
        move = random_move([(0,0),(2,0),(0,2),(2,2)], 0)
        if move != None:
            level = self.board.makeMove(move[0],move[1],self.robotPiece)
            self.place_piece(move[0], move[1], level)
            return      
                     
        
         # Try to take the middle center, if it's free.
        if not self.board.getPiece(1,1,1):
            level = self.board.makeMove(1,1,self.robotPiece)
            self.place_piece(1,1,level)
            return               
       
        
        # Try to take any corner, if it's free.
        move = random_move([(0,0),(2,0),(0,2),(2,2)], 2)
        if move != None:
            level = self.board.makeMove(move[0],move[1],self.robotPiece)
            self.place_piece(move[0], move[1], level)
            return     
        
        # Move on one of the sides.  
        move = random_move([(0,1), (1,0), (2,1), (1,2)], 2)
        level = self.board.makeMove(move[0],move[1],self.robotPiece)
        self.place_piece(move[0], move[1], level)

    


                                            # Tracking Stuff
#--------------------------------------------------------------------------------------------------------------------------------------------#

    def get_contours(self,i,hsv):
        mask = cv2.inRange(hsv, np.array(self.objects[i][0:3]), np.array(self.objects[i][3:6]))
        # filter and fill the mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(self.objects[i][6],self.objects[i][6]))
        mask2 = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)

        ret,thresh = cv2.threshold(mask2,127,255,0)
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def detect_and_draw(self,imgmsg):
    
        img = CvBridge().imgmsg_to_cv2(imgmsg, desired_encoding='passthrough')

        # for background subtraction
        self.imgmsg = imgmsg
        
        
        #-------------------------------------- HOUGH CIRCLES ------------------------------------------------#
           # for detecting the board        
             
        cimg = img[:,:,0:3]
        cimg = cv2.cvtColor(cimg,cv2.COLOR_BGR2GRAY)
        cimg = cv2.medianBlur(cimg,5)
        #cimg = cv2.cvtColor(cimg,cv2.COLOR_GRAY2BGR)

        circles = cv2.HoughCircles(cimg,cv.CV_HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=10,maxRadius=20)
        
        # ensures all 4 corners are visible and that point storing is enabled
        if circles != None and self.storePointPix:# and len(circles[0,:]) > 3:
            circles = np.uint16(np.around(circles))
            centers = []
            for i in circles[0,:]:
                centers.append([i[0],i[1],i[2]])
                
            dis_TR = 100000
            dis_TL = 100000
            dis_BR = 100000
            dis_BL = 100000
            
            # calculate which corners the detected circles are (closest distance)
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
            
            # draw circles around detected circles
            cv2.circle(img,(TR[0],TR[1]),TR[2],(0,255,0),2)
            cv2.circle(img,(TR[0],TR[1]),2,(0,0,255),3)
            cv2.circle(img,(TL[0],TL[1]),TL[2],(0,0,255),2)
            cv2.circle(img,(TL[0],TL[1]),2,(0,0,255),3)
            cv2.circle(img,(BR[0],BR[1]),BR[2],(255,0,0),2)
            cv2.circle(img,(BR[0],BR[1]),2,(0,0,255),3)
            cv2.circle(img,(BL[0],BL[1]),BL[2],(150,30,200),2)
            cv2.circle(img,(BL[0],BL[1]),2,(0,0,255),3)
                        
            # distances between the grid space centres
            dx = (float(TR[0])-float(TL[0]))/4
            dy = (float(TR[1])-float(TL[1]))/4
            dx2 = (float(BR[0])-float(BL[0]))/4
            dy2 = (float(BR[1])-float(BL[1]))/4
                        
            # middle row beginning and end points      
            MLx = TL[0]+int(np.round((float(BL[0])-float(TL[0]))/2))
            MLy = TL[1]+int(np.round((float(BL[1])-float(TL[1]))/2))
            MRx = TR[0]+int(np.round((float(BR[0])-float(TR[0]))/2))
            MRy = TR[1]+int(np.round((float(BR[1])-float(TR[1]))/2))
            
            # distances between the grid space centres of the middle row
            dx3 = (MRx - MLx)/4
            dy3 = (MRy - MLy)/4

            self.pixelPoints = range(self.boardX **2)    
            for i in range(0,self.boardX):
                top = (TL[0]+int(np.round(dx*(i+1))),TL[1]+int(np.round(dy*(i+1))))     # top row
                mid = (MLx+int(np.round(dx3*(i+1))),MLy+int(np.round(dy3*(i+1))))       # middle row
                bot = (BL[0]+int(np.round(dx2*(i+1))),BL[1]+int(np.round(dy2*(i+1))))   # bottom row
                self.pixelPoints[i] = top                                         
                self.pixelPoints[i+3] = mid                                       
                self.pixelPoints[i+6] = bot                                        
                cv2.circle(img,top,2,(80*(i+1),0,0),3)
                cv2.circle(img,mid,2,(0,80*(i+1),0),3)
                cv2.circle(img,bot,2,(0,0,80*(i+1)),3)
                
                
            if len(centers) == 4:
                self.detectedAllCorners = True

        #--------------------------------------- BLOB DETECTION ----------------------------------------------#
        # for detecting robot blocks
        # Blur each frame
        simg = cv2.GaussianBlur(img,(5,5),0)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(simg, cv2.COLOR_BGR2HSV)
        i = self.robotPieceIndex
        #for i in range(1,len(self.objects)+1):
        contours = self.get_contours(i, hsv)
        my_contours = []
        
        minDist = 10000
        for cnt in contours:
            if cv2.contourArea(cnt)>1400:
                    my_contours.append(cnt)
                    x,y,w,h = cv2.boundingRect(cnt)
                    if self.displayContours:
                        cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
                    cx = x + (w/2) 
                    cy = y + (h/2)    
                    dist = self.distance_to((cx,cy),(self.width,self.height)) # target the most-right-bottom blob
                    if dist < minDist:                           
                        minDist = dist
                        self.x[i-1] = cx
                        self.y[i-1] = cy
                            
        if self.displayContours:
            cv2.drawContours(img,my_contours,-1,(0,255,0),3)

        
        if self.displayCamera:
            self.publish_img(img)
        cv2.imshow('RGB',img)
        k = cv2.waitKey(5) & 0xFF

        # ---------------------------------------------------------------------------------------------------------#
    # create subscriber to the required camera
    def subscribe_to_camera(self, camera):
        if camera == "left":
            #callback = self.left_camera_callback
            camera_str = "/cameras/left_hand_camera/image"
        elif camera == "right":
            #callback = self.right_camera_callback
            camera_str = "/cameras/right_hand_camera/image"
        else:
            sys.exit("ERROR - subscribe_to_camera - Invalid camera")

        image_topic = rospy.resolve_name(camera_str)      

        # subscribe to detection topic
        rospy.Subscriber(image_topic, Image, self.detect_and_draw)
        
        

    # used to place camera over target object
    def block_iterate(self, iteration, block_centre):
        # print iteration number
        #print "ITERATION ", iteration

        # find displacement of target from centre of image
        pixel_dx    = (self.width / 2) - block_centre[0]
        pixel_dy    = (self.height / 2) - block_centre[1]
        pixel_error = math.sqrt((pixel_dx * pixel_dx) + (pixel_dy * pixel_dy))
        error       = float(pixel_error * self.cam_calib * self.block_pickup_height)

        x_offset = - pixel_dy * self.cam_calib * self.block_pickup_height
        y_offset = - pixel_dx * self.cam_calib * self.block_pickup_height
        #print 'x_offset:',x_offset
        #print 'y_offset:',y_offset
        # update pose and find new block location data
        self.update_pose(x_offset, y_offset)

        # find displacement of target from centre of image
        pixel_dx    = (self.width / 2) - block_centre[0]
        pixel_dy    = (self.height / 2) - block_centre[1]
        pixel_error = math.sqrt((pixel_dx * pixel_dx) + (pixel_dy * pixel_dy))
        error       = float(pixel_error * self.cam_calib * self.block_pickup_height)

        return block_centre, error

    def config_camera(self, camera, x_res, y_res):
        if camera == "left":
            cam = baxter_interface.camera.CameraController("left_hand_camera")
        elif camera == "right":
            cam = baxter_interface.camera.CameraController("right_hand_camera")
        else:
            sys.exit("ERROR - open_camera - Invalid camera")

        # set camera parameters
        cam.resolution          = (int(x_res), int(y_res))
        cam.exposure            = -1             # range, 0-100 auto = -1
        cam.gain                = -1             # range, 0-79 auto = -1
        cam.white_balance_blue  = -1             # range 0-4095, auto = -1
        cam.white_balance_green = -1             # range 0-4095, auto = -1
        cam.white_balance_red   = -1             # range 0-4095, auto = -1

    
    def detect_pieces(self):

            
        def subtract_background():            
            oldImg = CvBridge().imgmsg_to_cv2(self.bgsubImgmsg, desired_encoding='passthrough')
            img = CvBridge().imgmsg_to_cv2(self.imgmsg, desired_encoding='passthrough')
            
            fgbg = cv2.BackgroundSubtractorMOG()
        
            oldImg = oldImg[:,:,0:3]
            fgmask = fgbg.apply(oldImg)

            img = img[:,:,0:3]
            fgmask = fgbg.apply(img)

            img1_bg = cv2.bitwise_and(img,img,mask = fgmask)

            # Blur image
            simg = cv2.GaussianBlur(img1_bg,(5,5),0)

            # Convert BGR to HSV
            hsv = cv2.cvtColor(simg, cv2.COLOR_BGR2HSV)

            contours = self.get_contours(self.playerPieceIndex, hsv)
            areaMax = 0.0
            cx = 0
            cy = 0
            for cnt in contours:
                if cv2.contourArea(cnt)>1400 and cv2.contourArea(cnt)>areaMax:
                        areaMax = cv2.contourArea(cnt)
                        x,y,w,h = cv2.boundingRect(cnt)
                        cv2.rectangle(img1_bg,(x,y),(x+w,y+h),(0,0,255),2)
                        cx = x + (w/2) # int(M['m10']/M['m00'])
                        cy = y + (h/2) # int(M['m01']/M['m00'])    
            #cv2.line(img1_bg,(cx,cy-20),(cx,cy+20),(255,50,100),3)
            #cv2.line(img1_bg,(cx-20,cy),(cx+20,cy),(255,50,100),3)

            #cv2.imshow('contoured',img1_bg)
            pos = (cx,cy)
            if pos != (0,0):
                self.publish_img(img1_bg)          # send to face
                minDist = float('inf')
                x = 10
                y = 10
                for row in range(self.boardY):
                    for col in range(self.boardX):
                        dist = self.distance_to(pos,self.board.getPixel(row,col)) 
                        if dist < minDist:
                            minDist = dist
                            x = row
                            y = col
                self.board.makeMove(x,y,self.playerPiece)
                self.bgsubImgmsg = self.imgmsg              # update stored image
                return True  
            else:
                return False
        
        def check_circles():
            centers = []
            for frames in range(self.numCircleFrames):
                img = CvBridge().imgmsg_to_cv2(self.imgmsg, desired_encoding='passthrough')
                
                # for circle pieces        
                cimg = img[:,:,0:3]
                cimg = cv2.cvtColor(cimg,cv2.COLOR_BGR2GRAY)
                cimg = cv2.medianBlur(cimg,5)
                

                circles = cv2.HoughCircles(cimg,cv.CV_HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=31,maxRadius=40)
                
                if circles != None:
                    circles = np.uint16(np.around(circles))
                
                    for i in circles[0,:]:
                        center = (i[0],i[1])
                        if center not in centers:
                            centers.append(center)
                        cv2.circle(img,(i[0],i[1]),i[2],(0,255,255),3)
                        cv2.circle(img,(i[0],i[1]),2,(0,0,255),3)
                                    
                    self.publish_img(img)
                if frames > 0:
                    rospy.sleep(1)
                 

            for row in range(self.boardY):
                for col in range(self.boardX):
                    if self.board.getHighPiece(row,col) == self.playerPiece:    # check places with a player piece in them
                        dists = []
                        hasCircle = self.board.getCircle(row,col)
                        for center in centers:
                            dist = self.distance_to(center,self.board.getPixel(row,col))
                            dists.append(dist)
                        if not dists:
                            if hasCircle:
                                self.board.makeMove(row,col,self.playerPiece,False)
                                self.bgsubImgmsg = self.imgmsg                  # update stored image
                                return True 
                        else:
                            # check to see if a circle piece has been played 
                            if min(dists) < self.spacePixelDist and not hasCircle:
                                self.board.makeMove(row,col,self.playerPiece,True)
                                self.bgsubImgmsg = self.imgmsg                  # update stored image
                                return True 
                            # check to see if a circle piece was removed         
                            elif min(dists) > self.spacePixelDist and hasCircle:
                                self.board.makeMove(row,col,self.playerPiece,False)
                                self.bgsubImgmsg = self.imgmsg                  # update stored image
                                return True                                          
            return False

        self.pose = self.board.getPosition(1,1) 
        self.baxter_ik_move(self.limb, self.pose) 
        nomove = True
        while nomove:
            speak('Your turn')
            self.send_image('yourturn')
            if subtract_background() or check_circles():     #then there is a change
                rospy.sleep(3)
                return
            self.send_image('3')
            self.send_image('2')
            self.send_image('1')
            self.send_image('go')
            rospy.sleep(2)
            if subtract_background() or check_circles():     #then there is a change
                rospy.sleep(3)
                return
                
    def distance_to(self,i,j):
        a = (i[0]-j[0]) 
        b = (i[1]-j[1])
        c2 = (a*a) + (b*b)
        return math.sqrt(c2)
                                                # Movement Functions
#--------------------------------------------------------------------------------------------------------------------------------------------#  

    def place_piece(self, x, y, level):
        self.start_position() 
        self.toggleDisplayFeed(1,True)   
        self.find_object() 
        self.send_image('default')               
        self.pickup_object()                        
        self.drop_on(self.board.getPosition(x,y),level+1)    
                        

    def start_position(self):
        self.setSpeed(self.normal)
        # move back to start position
        self.pose = [self.start_pos_x, self.start_pos_y, self.start_pos_z,     \
                 self.roll, self.pitch, self.yaw]
        self.baxter_ik_move(self.limb, self.pose)

    def find_object(self,BoardMode=False):
        # iterates to find closest pose above object and returns it
        
        if not BoardMode:
            tolerance = self.block_tolerance
            i = self.robotPieceIndex-1
        else:
            tolerance = self.board_tolerance
            i = int(len(self.pixelPoints)/2)        # middle pixel point index of the pixelpoint array
        
        error     = 2 * tolerance
        iteration = 1
        
        # iterate until arm over centre of target
        while error > tolerance or not self.detectedAllCorners:
            if not BoardMode:
                block_centre = (self.x[i],self.y[i])
            else:
                if self.pixelPoints[i] == i:        # if a middle pixel point has not been generated
                    continue
                block_centre = (self.pixelPoints[i][0],self.pixelPoints[i][1])
            block_centre, error = self.block_iterate(iteration,       \
                                      block_centre)
            iteration              += 1
            
        return self.pose 

    def pickup_object(self):
        self.setSpeed(self.slow)
        # save return pose
        origin = self.pose
        # move down to pick up object
        self.pose = (self.pose[0] + self.cam_x_offset,
                self.pose[1] + self.cam_y_offset,
                self.pose[2] + (self.block_grip_height - self.block_pickup_height),
                self.pose[3],
                self.pose[4],
                self.pose[5])
        self.baxter_ik_move(self.limb, self.pose)
        self.gripper.close()
        # return to origin
        self.baxter_ik_move(self.limb, origin)
        self.setSpeed(self.normal)

    def drop_on(self,target,level):
        # drop object onto target below obj pose param
        self.pose = target  
        self.baxter_ik_move(self.limb, self.pose)    
        self.setSpeed(self.slow)
        self.pose = (self.pose[0] + self.cam_x_offset,
                self.pose[1] + self.cam_y_offset,
                self.pose[2] + (self.block_grip_height - (self.distance - (self.block_height * level))),
                self.pose[3],
                self.pose[4],
                self.pose[5])
        self.baxter_ik_move(self.limb, self.pose)
        self.gripper.open()
        self.pose = target  
        self.baxter_ik_move(self.limb, self.pose)
        self.setSpeed(self.normal)
    
    def locate_board(self):
        self.setSpeed(self.slow)
        centre = self.find_object(True)
        
        # stop storing pixel points at centre position and update Board class model
        self.storePointPix = False    
        counter = 0
        centrePix = self.pixelPoints[4]
        for row in range(self.boardY):
            for col in range(self.boardX):
                self.board.setPixel(row,col,self.pixelPoints[counter])
                #print row,col, self.pixelPoints[counter]
                self.board.setPosition(row,col,(centre[0] + ((self.pixelPoints[counter][1] - centrePix[1]) * self.cam_calib * self.distance),
                                                centre[1] + ((self.pixelPoints[counter][0] - centrePix[0]) * self.cam_calib * self.distance),
                                                centre[2],
                                                centre[3],
                                                centre[4],
                                                centre[5]))
       
                counter += 1
        
        self.bgsubImgmsg = self.imgmsg

    
    # update pose in x and y direction
    def update_pose(self,dx, dy):
        x = self.pose[0] + dx
        y = self.pose[1] + dy
        self.pose = [x, y, self.pose[2], self.roll, self.pitch, self.yaw]
        self.baxter_ik_move(self.limb, self.pose)
        
    def baxter_ik_move(self, limb, rpy_pose):

        quaternion_pose = conversions.list_to_pose_stamped(rpy_pose, "base")
        limb_interface = baxter_interface.Limb(limb)
        node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id="base")

        ik_request.pose_stamp.append(quaternion_pose)
        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException), error_message:
            rospy.logerr("Service request failed: %r" % (error_message,))
            sys.exit("ERROR - baxter_ik_move - Failed to append pose")

        if ik_response.isValid[0]:
            #print("PASS: Valid joint configuration found")
            # convert response to joint position control dictionary
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            # move limb
            if self.limb == limb:

                self.limb_interface.move_to_joint_positions(limb_joints)
            else:
                self.other_limb_interface.move_to_joint_positions(limb_joints)
        else:
            # display invalid move message on head display
            self.send_image('errorjoint')
            self.pose = [self.pose[0], self.pose[1], self.pose[2], self.pose[3], self.pose[4]-(0.01 * math.pi), self.pose[5]]
            print 'adjusting'
            self.baxter_ik_move(limb,self.pose)
            self.send_image('default')
            #sys.exit("ERROR - baxter_ik_move - No valid joint configuration found")

        if self.limb == limb:               # if working arm
            quaternion_pose = self.limb_interface.endpoint_pose()
            position        = quaternion_pose['position']

            # if working arm remember actual (x,y) position achieved
            self.pose = [position[0], position[1],                                \
                         self.pose[2], self.pose[3], self.pose[4], self.pose[5]]
                             
    def setSpeed(self,speed):
        self.limb_interface.set_joint_position_speed(speed)
    

                                                # Setup Functions
# -------------------------------------------------------------------------------------------------------------------------#


def get_distance_data():

    # read the setup parameters from setup.dat
    file_name = directory + "/setup.dat"

    try:
        f = open(file_name, "r")
    except ValueError:
        sys.exit("ERROR: golf_setup must be run before golf")

    #find limb
    s = string.split(f.readline())
    if len(s) >= 3:
        if s[2] == "left" or s[2] == "right":
            limb = s[2]
        else:
            sys.exit("ERROR: invalid limb in %s" % file_name)
    else:
        sys.exit("ERROR: missing limb in %s" % file_name)

    #find distance to table
    s = string.split(f.readline())
    if len(s) >= 3:
        try:
            distance = float(s[2])
        except ValueError:
            sys.exit("ERROR: invalid distance in %s" % file_name)
    else:
        sys.exit("ERROR: missing distance in %s" % file_name)

    return limb, distance

def get_object_data():
    
    Objects_pointer = open(directory+'/objects.txt', 'r')
    OBJ = {}
    for line in Objects_pointer:
        line = line.strip(',\n')
        if line == 'END':
            break
        fields = line.split(',')
        fields = map(int, fields)
        OBJ[fields[0]] = fields[1:len(fields)]


    return OBJ





if __name__ == "__main__":
    main()
