#!/usr/bin/env python

from __future__ import division
from __future__ import print_function

import math
import os
import Queue
import rospy
import rospkg
import serial

from sensor_msgs.msg import JointState
from flo_humanoid.msg import JointTarget
from read_from_bolide import BolideReader

class BolideController(object):
    def __init__(self):
        rospy.init_node('robot_manager')

        rospack = rospkg.RosPack()

        port = rospy.get_param('robot_port', '/dev/bolide')
        self.ser = None
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.05)
        except serial.SerialException:
            rospy.logerr('failed to connect to bolide')
            return
        rospy.loginfo('connected to robot')

        self.joint_publisher = rospy.Publisher(
            'joint_states', JointState, queue_size=1)

        self.command_reader = rospy.Subscriber(
            'target_joint_states', JointTarget, self.new_command)

        self.reader = BolideReader(self.ser)
        package_path = rospack.get_path('flo_humanoid')
        default_config_fn = os.path.join(package_path, 'config', 'joints')
        config_fn = rospy.get_param('robot_joint_config',
                                    default_config_fn)
        self.joint_config = dict()
        self.available_motor_ids = []
        self.available_motor_names = []
        with open(config_fn) as cfile:
            headings = None
            for row in cfile:
                if not headings:
                    headings = row.split()
                else:
                    new_data = row.split()
                    new_data_dict = {key: value for key, value in
                                     zip(headings, new_data)}
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
        """If there are motion tasks to do, do those, otherwise, check the
        robot's pose"""
        while not rospy.is_shutdown():
            if self.tasks.empty():
                self.get_pose()
            else:
                return None
            self.rate.sleep()

    def get_pose(self):
        """Get the pose of the robot and publish it to the joint state"""
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
            rad_position = (raw_position - int(
                self.joint_config[id]['neutral'])) * int(
                    self.joint_config[id]['inversion'])*2*math.pi/1023
            names.append(self.joint_config[id]['name'])
            positions.append(rad_position)
        new_msg = JointState()
        new_msg.name = names
        new_msg.position = positions
        new_msg.header.stamp = rospy.Time.now()
        self.joint_publisher.publish(new_msg)

    def new_command(self, msg):
        """Take a new message from the joint command topic and add it to the task queue

        :param msg: The message that is being passed in that we should parse
        """
        self.tasks.put(msg)


if __name__ == "__main__":
    controller = BolideController()
