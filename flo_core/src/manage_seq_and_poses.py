#! /usr/bin/env python

from __future__ import division
from __future__ import print_function


from db import DB
import sys
import select
import termios
import tty
import pdb
import json

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from flo_humanoid.msg import JointTarget


class Programmer(object):

    def __init__(self):
        self.commands = {'r': ('record a pose on the right arm', self.record_pose, 'right'),
                         'l': ('record a pose on the left arm', self.record_pose, 'left'),
                         'q': ('quit the program', self.quit, None),
                         'p': ('enumerate all saved poses', self.enumerate_poses, None),
                         'h': ('print help message', self.help, None),
                         'e': ('move to a pose on the right arm', self.move, 'right'),
                         'o': ('move to a pose on the left arm', self.move, 'left'),
                         'c': ('relax the motors', self.relax, None),
                         's': ('search for a pose', self.search_poses, None)
                         }

        rospy.init_node('motion_seq_programmer')
        self.run = True

        db_path = rospy.get_param("database_location")
        self.db = DB(db_path)

        rospy.Subscriber('joint_states', JointState, self.new_joint_data)
        self.current_joint_data = None

        self.target_pub = rospy.Publisher(
            'target_joint_states', JointTarget, queue_size=4)
        self.control_pub = rospy.Publisher(
            'motor_commands', String, queue_size=1)

    def keyboard_interface(self):
        rate = rospy.Rate(100)
        rospy.sleep(3)
        self.old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.help()
        while not rospy.is_shutdown() and self.run:
            if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
                new_input = sys.stdin.read(1)
                if new_input in self.commands.keys():
                    arg = self.commands[new_input][2]
                    if arg:
                        self.commands[new_input][1](arg)
                    else:
                        self.commands[new_input][1]()
                    print('---------------\n')
                else:
                    print('unkown key pressed, press h for help')
                    print('---------------\n')
            rate.sleep()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)

    def new_joint_data(self, data):
        self.current_joint_data = data

    def record_pose(self, side):
        if self.current_joint_data:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)
            description = raw_input(
                "Please provide a description of the pose on the {} arm, enter cancel to cancel:\n".format(side))
            if description == 'cancel':
                tty.setcbreak(sys.stdin.fileno())
                print('cancelled')
                return
            joints_of_interest = [idx for (idx, el) in enumerate(
                self.current_joint_data.name) if side in el]
            names = [self.current_joint_data.name[idx]
                     for idx in joints_of_interest]
            pos = [self.current_joint_data.position[idx]
                   for idx in joints_of_interest]
            clean_names = [itm[len(side)+1:] for itm in names]
            self.db.add_pose(description, pos, clean_names)
            tty.setcbreak(sys.stdin.fileno())
            print('saved')
        else:
            print('no joint data available')

    def enumerate_poses(self):
        print('pose ids and descriptions:')
        for row in self.db.ex('select id, description from poses'):
            print(row)

    def help(self):
        for key in self.commands.keys():
            print('key: {}\t\t{}'.format(key, self.commands[key][0]))

    def quit(self):
        self.run = False

    def move(self, side):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)
        description = raw_input(
            "Please provide the ID of the pose to move arm {} to, type cancel to cancel:\n".format(side))
        if description == 'cancel':
            tty.setcbreak(sys.stdin.fileno())
            print('cancelled')
            return
        curs = self.db.ex('select * from poses where id = ?', int(description))
        data = curs.fetchone()
        msg = JointTarget()
        msg.name = [side+'_'+el for el in json.loads(data['names'])]
        msg.position = json.loads(data['angles'])
        msg.target_completion_time = 2
        tty.setcbreak(sys.stdin.fileno())
        self.target_pub.publish(msg)
        self.control_pub.publish('move')

    def relax(self):
        self.control_pub.publish('relax')
        print('relaxing motors')

    def search_poses(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)
        target = raw_input("Input the desired search term:\n")
        tty.setcbreak(sys.stdin.fileno())

        found = False

        for row in self.db.ex('select id, description from poses where description like ?',
                              '%'+target+'%'):
            print(row)
            found = True
        if not found:
            print('no records found')


if __name__ == "__main__":
    pr = Programmer()
    pr.keyboard_interface()
