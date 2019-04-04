#!/usr/bin/env python

import csv
import rospy
import rospkg
import os
import Queue
import serial
import math

from read_from_bolide import BolideReader 
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from flo_humanoid.msg import JointTarget

class BolideController:
    def __init__(self):
        rospy.init_node('robot_manager')

        self.rospack = rospkg.RosPack()
        
        self.port = rospy.get_param('robot_port','/dev/bolide')
        self.ser = None
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=0.05)
        except:
            rospy.logerr('failed to connect to bolide')
            return
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
        self.available_motor_ids = []
        self.available_motor_names = []
        with open(self.config_fn) as cfile:
            headings = None
            for row in cfile:
                if not headings:
                    headings = row.split()
                else:
                    new_data = row.split()
                    new_data_dict = {key: value for key, value in zip(headings,new_data)}
                    this_address = int(new_data_dict['address'])
                    this_name = new_data_dict['name']
                    self.joint_config[this_address] = new_data_dict
                    self.joint_config[this_name] = new_data_dict
                    self.available_motor_ids.append(this_address)
                    self.available_motor_names.append(this_name)

        self.rate = rospy.Rate(3)
        self.tasks = Queue.Queue()
        self.read_loop()

    def read_loop(self):
        while not rospy.is_shutdown():
            if self.tasks.empty():
                self.get_pose()
            else:
                return None
            self.rate.sleep()
                    
    def get_pose(self):
        position = self.reader.read_data('pos')
        if not position:
            rospy.logerr('couldn\'t get postion data')
            return
        print(position)
        names = []
        positions = []
        for id in self.available_motor_ids:
            raw_position = position[id]
            # import pdb
            # pdb.set_trace()
            rad_position = (raw_position - int(self.joint_config[id]['neutral'])) * int(self.joint_config[id]['inversion'])*2*math.pi/1023
            names.append(self.joint_config[id]['name'])
            positions.append(rad_position)
        new_msg = JointState()
        new_msg.name = names
        new_msg.position = positions
        new_msg.header.stamp = rospy.Time.now()
        self.joint_publisher.publish(new_msg)
        #TODO: take position data and turn it into something that can be broadcast

    def new_command(self, msg):
        self.tasks.put(msg)

if __name__ == "__main__":
    controller=BolideController()
