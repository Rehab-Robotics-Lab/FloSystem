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


class PositionRecorder(object):

    def __init__(self):
        self.commands = {'r': ('move to a pose on the right arm', self.move, 'right'),
                         'l': ('move to a pose on the left arm', self.move, 'left'),
                         'q': ('quit the program', self.quit, None),
                         'e': ('enumerate all saved poses', self.enumerate, None),
                         'h': ('print help message', self.help, None)
                         }

        rospy.init_node('pose_teleop')
        self.run = True

        db_path = rospy.get_param("database_location")
        self.db = DB(db_path)

        self.target_pub = rospy.Publisher(
            'target_joint_states', JointTarget, queue_size=4)
        self.control_pub = rospy.Publisher(
            'motor_commands', String, queue_size=1)

        # self.old_attr = termios.tcgetattr(sys.stdin)
        # self.move('right')

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
                else:
                    print('unkown key pressed, press h for help')
            rate.sleep()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)

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

    def enumerate(self):
        print('pose ids and descriptions:')
        for row in self.db.ex('select id, description from poses'):
            print(row)

    def help(self):
        for key in self.commands.keys():
            print('key: {}\t\t{}'.format(key, self.commands[key][0]))

    def quit(self):
        self.run = False


if __name__ == "__main__":
    pr = PositionRecorder()
    pr.keyboard_interface()
