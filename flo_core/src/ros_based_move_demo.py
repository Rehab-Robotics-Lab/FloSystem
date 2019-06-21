#!/usr/bin/env python

from __future__ import division
from __future__ import print_function

import sqlite3


from std_msgs.msg import String
from sensor_msgs.msg import JointState
from flo_humanoid.msg import JointTarget

class PositionRecorder(object):
    commands = {'r': self.record, 'l': self.list, 'q': self.quit}

    def __init__(self):
        rospy.init_node('robot_pos_recorder')
        db_path = rospy.get_param("positions_file")
        rospy.Subscriber('joint_states', JointState, self.new_joint_data)
        self.current_joint_data = null
        self.db_conn = sqlite3.connect(db_path)
        while not rospy.is_shutdown():


    def new_joint_data(self, data):
        self.current_joint_data = data

    def record(self):
        with open(self.file_path) as json_file:

