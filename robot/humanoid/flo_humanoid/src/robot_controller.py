#!/usr/bin/env python

import csv
import rospy
import rospkg
import os

from read_from_bolide import BolideReader 
from sensor_msgs.msg import JointState
from flo_humanoid.msg import JointTarget

class BolideController:
    def __init__(self):
        rospy.init_node('robot_manager')

        self.rospack = rospkg.RosPack()
        
        self.port = rospy.get_param('robot_port','/dev/bolide')
        self.ser = None
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=0.2)
        except:
            rospy.logerr('failed to connect to bolide')
        rospy.loginfo('connected to robot')

        self.joint_publisher = rospy.Publisher(
                'joint_states', JointState, queue_size=1)

        self.command_reader = rospy.Subscriber('target_joint_states',JointTarget,self.new_command)

        self.reader = BolideReader(self.ser)
        #TODO: read in a paramater file describing servo offsets
        self.package_path = self.rospack.get_path('flo_humanoid')
        default_config_fn = os.path.join(self.package_path,'config','joints')
        self.config_fn = rospy.get_param('robot_joint_config',default_config_fn)
        self.joint_config = dict()
        with open(self.config_fn) as cfile:
            headings = None
            for row in cfile:
                if not headings:
                    headings = row.split()
                else:
                    new_data = row.split()
                    new_data_dict = {key: value for key, value in zip(headings,new_data)}
                    self.joint_config[new_data_dict['address']] = new_data_dict
                    self.joint_config[new_data_dict['name']] = new_data_dict
                    
        return

        # self.params = 

    def get_pos(self):
        position = self.reader.read_data('pos')
        if not position:
            rospy.logerr('couldn\'t get postion data')
            return

        #TODO: take position data and turn it into something that can be broadcast

    def new_command(self, msg):
        return None
        #TODO: take in a commanded joint state and target system time to achieve it, interpolate it out to make it happen and send the command to the robot


if __name__ == "__main__":
    controller=BolideController()
