#!/usr/bin/env python

import rospy
import baxter_interface


rospy.init_node('Pickup_Drop_Test')

limb = baxter_interface.Limb('right')
grip = baxter_interface.Gripper('right')

print (raw_input('Place Right Arm above object and Press Enter'))

pos0 = limb.joint_angles()

print (raw_input('Place Right Arm at object and Press Enter'))
pos1 = limb.joint_angles()

print (raw_input('Place Right Arm above destination and Press Enter'))

pos2 = limb.joint_angles()

print (raw_input('Place Right Arm at destination and Press Enter'))

pos3 = limb.joint_angles()

print (raw_input('Place Right Arm at star position and Press Enter to initialise!'))
ans = 'y'
while (ans == 'y'):
	grip.open()
	limb.move_to_joint_positions(pos0)
	limb.move_to_joint_positions(pos1)
	grip.close()
	limb.move_to_joint_positions(pos0)
	limb.move_to_joint_positions(pos2)
	limb.move_to_joint_positions(pos3)
	grip.open()
	limb.move_to_joint_positions(pos2)
	limb.move_to_joint_positions(pos0)
		
	ans = (raw_input('Repeat? (y/n) '))
	



