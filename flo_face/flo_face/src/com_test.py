#!/usr/bin/env python

import rospy
import random
from flo_face.msg import FaceState

rospy.init_node('com_tester')
rate = rospy.Rate(3)

pub = rospy.Publisher('face_state',FaceState,queue_size=1)

while not rospy.is_shutdown():
    mouth = [1 if random.random() > .5 else 0 for e in range(16*8)] 
    eye = [1 if random.random() > .5 else 0 for e in range(8*8)]
    msg = FaceState()
    msg.left_eye = eye
    msg.right_eye = eye 
    msg.mouth = mouth
    pub.publish(msg)
    rospy.loginfo('publishing face to test')
    rate.sleep()
