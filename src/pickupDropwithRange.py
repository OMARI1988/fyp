#!/usr/bin/env python

import rospy
import baxter_interface
import math
from std_msgs.msg import String
from sensor_msgs.msg import Range
from baxter_core_msgs.msg import EndpointState
from moveit_commander import conversions

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

#global range1
#range1 = 0

def ik_solver_request(input_limb, input_pose):
    print "IK solver request:"

    #input error checking
    if len(input_pose) == 6:
        quaternion_pose = conversions.list_to_pose_stamped(input_pose, "base")
    elif len(input_pose) == 7:
        quaternion_pose = input_pose
    else:
        print """Invalid Pose List:
    Input Pose to function: ik_solver_request must be a list of:
    6 elements for an RPY pose, or
    7 elements for a Quaternion pose"""
        return

    if input_limb == "right" or input_limb == "left":
        limb = input_limb
    else:
        print """Invalid Limb:
    Input Limb to function: ik_solver_request must be a string:
    'right' or 'left'"""
        return

    #request/response handling
    node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    ik_service = rospy.ServiceProxy(node, SolvePositionIK)
    ik_request = SolvePositionIKRequest()
    ik_request.pose_stamp.append(quaternion_pose)
    try:
        rospy.wait_for_service(node, 5.0)
        ik_response = ik_service(ik_request)
    except (rospy.ServiceException, rospy.ROSException), error_message:
        rospy.logerr("Service request failed: %r" % (error_message,))
    if (ik_response.isValid[0]):
        print("PASS: Valid joint configuration found")
        #convert response to JP control dict
        limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
        return limb_joints
    else:
        print("FAILED: No valid joint configuration for this pose found")


def callbackRange(data):
	global rangeIR
	rangeIR = data.range

def callbackEndpoint(data):
	global positionEP
	positionEP = data.pose.position

if __name__ == '__main__':
	global rangeIR
	global positionEP
	pi = math.pi
	threshold = 0.12

	rospy.init_node('Pickup_Drop_Test')

    	rospy.Subscriber("/robot/range/right_hand_range/state", Range, callbackRange)
	rospy.Subscriber("/robot/limb/right/endpoint_state",EndpointState, callbackEndpoint)

	limb = baxter_interface.Limb('right')
	grip = baxter_interface.Gripper('right')

	
	# Move Arm Down Until above the object (height = threshold) 
	while rangeIR > threshold:
		
		print rangeIR
		positionEP.z -= 0.05
		pose = [positionEP.x, positionEP.y, positionEP.z, pi, 0, pi]
		angles = ik_solver_request('right', pose)
		baxter_interface.Limb('right').move_to_joint_positions(angles)

	# Close grip, wait a second then pick it up
	grip.close()
	#rospy.sleep(1)
	positionEP.z += 0.2
	pose = [positionEP.x, positionEP.y, positionEP.z, pi, 0, pi]
	angles = ik_solver_request('right', pose)
	baxter_interface.Limb('right').move_to_joint_positions(angles)

	# Wait 3 Seconds then drop it
	#rospy.sleep(3)
	grip.open()




