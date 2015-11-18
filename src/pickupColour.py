#!/usr/bin/env python

import roslib
import sys
import inspect, os
import rospy
import string
import sensor_msgs.msg
from std_msgs.msg import Header
from moveit_commander import conversions
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
import baxter_interface
from cv_bridge import CvBridge
import cv
import cv2
import numpy as np
import math

directory = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

# start positions
ball_tray_x = 0.50                        # x     = front back
ball_tray_y = 0.30                        # y     = left right
ball_tray_z = 0.15                        # z     = up down
roll        = -1.0 * math.pi              # roll  = horizontal
pitch       = 0.0 * math.pi               # pitch = vertical
yaw         = 0.0 * math.pi               # yaw   = rotation


pose = [ball_tray_x, ball_tray_y, ball_tray_z,     \
             roll, pitch, yaw]

block_height = 0.05
block_grip_height = 0.102
block_tolerance = 0.005

# camera parameters (NB. other parameters in open_camera)

cam_calib    = 0.0025                     # meters per pixel at 1 meter
#cam_x_offset = 0.045                    # original camera gripper offset
#cam_y_offset = -0.01
cam_x_offset = 0.025                      # camera gripper offset
cam_y_offset = -0.02
width        = 960                        # Camera resolution
height       = 600

def main():
	
	global x,y,z,rgb_flag,OBJ,br,distance,block_distance,limb,gripper
	br = CvBridge()
	rgb_flag = 0

    # get setup parameters
	limb, distance = get_distance_data()
	block_distance = distance - 0.05
	

	print "limb     = ", limb
	print "distance = ", distance
	
	# read colour data from file
	OBJ = get_object_data()
		
	x = np.zeros(shape=(len(OBJ)), dtype=int)
	y = np.zeros(shape=(len(OBJ)), dtype=int)
	z = np.zeros(shape=(len(OBJ)), dtype=float)

	rospy.init_node('rosColordetect')

	# set camera resolution
	config_camera(limb, width, height)

	# move to start position
	baxter_ik_move(limb, pose)

	gripper = baxter_interface.Gripper(limb)
	gripper.open()
	image_topic = rospy.resolve_name("/cameras/left_hand_camera/image")      
	
	# subscribe to detection topic
	rospy.Subscriber(image_topic, sensor_msgs.msg.Image, detect_and_draw)


	# wait for camera feedback (may not be needed)
	rospy.sleep(1)

	find_object()

	rospy.spin()

def find_object():

	global pose
	error     = 2 * block_tolerance
	iteration = 1
		
	# iterate until arm over centre of tray
	while error > block_tolerance:
		print (x[0],y[0])
		block_centre = (x[0],y[0])
		block_centre, error = block_iterate(iteration,       \
		                          block_centre)
		iteration              += 1
	
	# move down to pick up ball
	pose = (pose[0] + cam_x_offset,
		    pose[1] + cam_y_offset,
		    pose[2] + (block_grip_height - block_distance),
		    pose[3],
		    pose[4],
		    pose[5])
	baxter_ik_move(limb, pose)

	gripper.close()
	
	# move back to start position
	pose = [ball_tray_x, ball_tray_y, ball_tray_z,     \
             roll, pitch, yaw]
	baxter_ik_move(limb, pose)

	# drop object
	gripper.open()

	#sys.exit()
	
 # used to place camera over target object
def block_iterate(iteration, block_centre):
	# print iteration number
	print "ITERATION ", iteration

	# find displacement of target from centre of image
	pixel_dx    = (width / 2) - block_centre[0]
	pixel_dy    = (height / 2) - block_centre[1]
	pixel_error = math.sqrt((pixel_dx * pixel_dx) + (pixel_dy * pixel_dy))
	error       = float(pixel_error * cam_calib * block_distance)

	x_offset = - pixel_dy * cam_calib * block_distance
	y_offset = - pixel_dx * cam_calib * block_distance
	print 'x_offset:',x_offset
	print 'y_offset:',y_offset
	# update pose and find new block location data
	update_pose(x_offset, y_offset)

	# find displacement of target from centre of image
	pixel_dx    = (width / 2) - block_centre[0]
	pixel_dy    = (height / 2) - block_centre[1]
	pixel_error = math.sqrt((pixel_dx * pixel_dx) + (pixel_dy * pixel_dy))
	error       = float(pixel_error * cam_calib * block_distance)

	return block_centre, error

# update pose in x and y direction
def update_pose(dx, dy):
	global pose
	x = pose[0] + dx
	y = pose[1] + dy
	pose = [x, y, pose[2], roll, pitch, yaw]
	baxter_ik_move(limb, pose)

def detect_and_draw(imgmsg):
	global x,y,z,rgb_flag,OBJ,br
	
	img = br.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
	
  	# Take each frame
	simg = cv2.GaussianBlur(img,(5,5),0)

		# Convert BGR to HSV
	hsv = cv2.cvtColor(simg, cv2.COLOR_BGR2HSV)
	hsv2 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	for i in range(1,len(OBJ)+1):
	#Threshold the HSV image to get only orange colors
		mask = cv2.inRange(hsv, np.array(OBJ[i][0:3]), np.array(OBJ[i][3:6]))

		# filter and fill the mask
		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(OBJ[i][6],OBJ[i][6]))
		mask2 = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)

		moments = cv2.moments(mask2) 
		area = moments['m00'] 
		#there can be noise in the video so ignore objects with small areas 
		if(area > 10000): 
		  	#determine the x and y coordinates of the center of the object 
				#we are tracking by dividing the 1, 0 and 0, 1 moments by the area 
				x1 = int(moments['m10'] / area)
				y1 = int(moments['m01'] / area)
	
				x[i-1] = x1
				y[i-1] = y1
				cv2.line(img,(x1,y1-20),(x1,y1+20),(255,50,100),3)
				cv2.line(img,(x1-20,y1),(x1+20,y1),(255,50,100),3)
				cv2.putText(img,'obj'+str(i-1),(x1+20,y1+20), cv2.FONT_HERSHEY_SIMPLEX, .5,(0,0,0),1)
				rgb_flag = 1
		else:
				x[i-1] = 0
				y[i-1] = 0
				rgb_flag = 1
			

	cv2.imshow('RGB',img)
	#cv2.imshow('Mask',mask2)
	k = cv2.waitKey(5) & 0xFF

def baxter_ik_move(limb, rpy_pose):
        global pose
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
            print("PASS: Valid joint configuration found")
            # convert response to joint position control dictionary
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            # move limb
            if limb == limb:
			
                limb_interface.move_to_joint_positions(limb_joints)
            else:
                other_limb_interface.move_to_joint_positions(limb_joints)
        else:
            # display invalid move message on head display
            #splash_screen("Invalid", "move")
            # little point in continuing so exit with error message
            print "requested move =", rpy_pose
            sys.exit("ERROR - baxter_ik_move - No valid joint configuration found")

        if limb == limb:               # if working arm
            quaternion_pose = limb_interface.endpoint_pose()
            position        = quaternion_pose['position']

            # if working arm remember actual (x,y) position achieved
            pose = [position[0], position[1],                                \
                         pose[2], pose[3], pose[4], pose[5]]

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

def config_camera(camera, x_res, y_res):
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



if __name__ == "__main__":
	main()
