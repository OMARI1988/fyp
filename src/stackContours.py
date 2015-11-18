#!/usr/bin/env python

import roslib
import sys
import inspect, os
import rospy
import string
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from moveit_commander import conversions
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
import baxter_interface
from cv_bridge import CvBridge
import cv
import cv2
import numpy as np
import math

# init rospy node
rospy.init_node('pickup_and_stack')

# file directory
directory = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

def main():

	# get setup parameters
	limb, distance = get_distance_data()

	# read colour data from file
	colours = get_object_data()

	# create stack class instance
	stacker = stack(limb, distance, colours)	

	print "limb     = ", limb
	print "distance = ", distance

	#raw_input("Press Enter to start: ")

	# open the gripper
	stacker.gripper.open()

	# move to start position
	stacker.start_position()
	
	# wait for camera feedback
	rospy.sleep(1)

	obj1 = stacker.find_object(0)
	stacker.start_position()
	obj2 = stacker.find_object(1)
	stacker.pickup_object()  # TODO: add obj var as param
	stacker.start_position()
	stacker.stack_on(obj1)
	stacker.start_position()

	# drop object
	stacker.gripper.open()

	sys.exit()

	rospy.spin()

class stack():
	def __init__(self, arm, distance, colours):

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


		# start positions
		self.start_pos_x = 0.50                        # x     = front back
		self.start_pos_y = 0.30                        # y     = left right
		self.start_pos_z = 0.15                        # z     = up down
		self.roll        = -1.0 * math.pi              # roll  = horizontal
		self.pitch       = 0.0 * math.pi               # pitch = vertical   
		self.yaw         = 0.0 * math.pi               # yaw   = rotation


		self.pose = [self.start_pos_x, self.start_pos_y, self.start_pos_z,     \
					 self.roll, self.pitch, self.yaw]

		# distances
		self.distance = distance	
		self.block_height = 0.05
		self.block_pickup_height = self.distance - self.block_height
		self.block_stack_height = self.distance - (self.block_height * 2)
		self.block_grip_height = 0.092
		self.block_tolerance = 0.005

		# camera parameters (NB. other parameters in open_camera)
		self.cam_calib    = 0.0025                     # meters per pixel at 1 meter
		#self.cam_x_offset = 0.045                    # original camera gripper offset
		#self.cam_y_offset = -0.01
		self.cam_x_offset = 0.035                      # camera gripper offset
		self.cam_y_offset = -0.02
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

		# move other arm out of harms way
		if arm == "left":
			self.baxter_ik_move("right", (0.25, -0.50, 0.2, math.pi, 0.0, 0.0))
		else:
			self.baxter_ik_move("left", (0.25, 0.50, 0.2, math.pi, 0.0, 0.0))
#--------------------------------------------------------------------------------------------------------------------------------------------#


	def detect_and_draw(self,imgmsg):
	
		img = CvBridge().imgmsg_to_cv2(imgmsg, desired_encoding='bgr8')
		#self.IDBlock(img)
	
	  	# Take each frame
		simg = cv2.GaussianBlur(img,(5,5),0)

		# Convert BGR to HSV
		hsv = cv2.cvtColor(simg, cv2.COLOR_BGR2HSV)
		hsv2 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		conImage = ''
		for i in range(1,len(self.objects)+1):
		#Threshold the HSV image to get only orange colors
			mask = cv2.inRange(hsv, np.array(self.objects[i][0:3]), np.array(self.objects[i][3:6]))

			# filter and fill the mask
			kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(self.objects[i][6],self.objects[i][6]))
			mask2 = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
			
			
			moments = cv2.moments(mask2) 
			if i < 3:
				cv2.imshow('Mask',mask2)
			
			#contours
			ret,thresh = cv2.threshold(mask2,127,255,0)
			contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
			my_contours = []
			xMax = 0
			for cnt in contours:
				if cv2.contourArea(cnt)>800:
						my_contours.append(cnt)
						M = cv2.moments(cnt)
						cy = int(M['m01']/M['m00'])
						cx = int(M['m10']/M['m00'])
						if cx > xMax:
							xMax = cx
							self.x[i-1] = cx
							self.y[i-1] = cy
						cv2.line(img,(cx,cy-20),(cx,cy+20),(255,50,100),3)
						cv2.line(img,(cx-20,cy),(cx+20,cy),(255,50,100),3)
			print "# contours:", len(my_contours)
			cv2.drawContours(img,my_contours,-1,(0,255,0),3)
		cv2.imshow('RGB',img)
		k = cv2.waitKey(5) & 0xFF

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
			print("PASS: Valid joint configuration found")
			# convert response to joint position control dictionary
			limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
			# move limb
			if self.limb == limb:

				self.limb_interface.move_to_joint_positions(limb_joints)
			else:
				self.other_limb_interface.move_to_joint_positions(limb_joints)
		else:
			# display invalid move message on head display
			#splash_screen("Invalid", "move")
			# little point in continuing so exit with error message
			print "requested move =", rpy_pose
			sys.exit("ERROR - baxter_ik_move - No valid joint configuration found")

		if self.limb == limb:               # if working arm
			quaternion_pose = self.limb_interface.endpoint_pose()
			position        = quaternion_pose['position']

			# if working arm remember actual (x,y) position achieved
			self.pose = [position[0], position[1],                                \
						 self.pose[2], self.pose[3], self.pose[4], self.pose[5]]

	# used to place camera over target object
	def block_iterate(self, iteration, block_centre):
		# print iteration number
		print "ITERATION ", iteration

		# find displacement of target from centre of image
		pixel_dx    = (self.width / 2) - block_centre[0]
		pixel_dy    = (self.height / 2) - block_centre[1]
		pixel_error = math.sqrt((pixel_dx * pixel_dx) + (pixel_dy * pixel_dy))
		error       = float(pixel_error * self.cam_calib * self.block_pickup_height)

		x_offset = - pixel_dy * self.cam_calib * self.block_pickup_height
		y_offset = - pixel_dx * self.cam_calib * self.block_pickup_height
		print 'x_offset:',x_offset
		print 'y_offset:',y_offset
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

	def start_position(self):
		# move back to start position
		self.pose = [self.start_pos_x, self.start_pos_y, self.start_pos_z,     \
		         self.roll, self.pitch, self.yaw]
		self.baxter_ik_move(self.limb, self.pose)

	def find_object(self,i):
		# iterates to find closest pose above object and returns it
		error     = 2 * self.block_tolerance
		iteration = 1
		
		# iterate until arm over centre of target
		while error > self.block_tolerance:
			print (self.x[i],self.y[i])
			block_centre = (self.x[i],self.y[i])
			block_centre, error = self.block_iterate(iteration,       \
				                      block_centre)
			iteration              += 1
			
		return self.pose 

	def pickup_object(self):
		# move down to pick up object
		# TODO: slow down arm speed here
		self.pose = (self.pose[0] + self.cam_x_offset,
				self.pose[1] + self.cam_y_offset,
				self.pose[2] + (self.block_grip_height - self.block_pickup_height),
				self.pose[3],
				self.pose[4],
				self.pose[5])
		self.baxter_ik_move(self.limb, self.pose)

		self.gripper.close()
	
	def stack_on(self,obj):
		# drop object on top of similar size object
		self.pose = obj  # may not need this
		self.baxter_ik_move(self.limb, self.pose)	# may need just obj as param
		
		self.pose = (self.pose[0] + self.cam_x_offset,
				self.pose[1] + self.cam_y_offset,
				self.pose[2] + (self.block_grip_height - self.block_stack_height),
				self.pose[3],
				self.pose[4],
				self.pose[5])
		self.baxter_ik_move(self.limb, self.pose)

		self.gripper.open()
		

	# update pose in x and y direction
	def update_pose(self,dx, dy):
		x = self.pose[0] + dx
		y = self.pose[1] + dy
		self.pose = [x, y, self.pose[2], self.roll, self.pitch, self.yaw]
		self.baxter_ik_move(self.limb, self.pose)
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
