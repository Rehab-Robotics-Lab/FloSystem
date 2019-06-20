#!/usr/bin/env python

from __future__ import division
from __future__ import print_function

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from flo_humanoid.msg import JointTarget

class PositionRecorder(object):
    def __init__(self):
        rospy.init_node('robot_pos_recorder')
        file_path=rospy.get_param("positions_file")
        rospy.su
