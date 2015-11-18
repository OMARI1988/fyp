#!/usr/bin/env python

import rospy
import sys

#import alsaaudio, wave

import numpy as np
#import psw
#import gapi
#import commands
from ros_mary_tts.srv import *
import baxter_interface
from baxter_interface import CHECK_VERSION
#from copy import deepcopy
from std_msgs.msg import (
    Empty,
    Bool,
)
#from robot_functions import *
#from baxter_demos.msg import obj_hypotheses,action

#------------------------------------------------------------------#
def speak(x):
    rospy.wait_for_service('ros_mary')
    try:
        add_two_ints = rospy.ServiceProxy('ros_mary',ros_mary)
        resp1 = add_two_ints(x)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#------------------------------------------------------------------#
#speech = gapi.Speech('sp')
COUNTER = 0

print 'Speech to text recognition starting ..'

#pub2 = rospy.Publisher('/action_from_voice', action, queue_size=1)
#pub2 = rospy.Publisher('obj_manipulation_voice', obj_hypotheses, queue_size=1)
rospy.init_node('Voice_recognition')
#mic = psw.Microphone()
print 'sampling...'
#sample = np.array(mic.sample(200))
#print 'done'

speak('Ready.')
#mic.listen(handler, sample.mean(), sample.std())
