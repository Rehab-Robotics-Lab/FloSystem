#! /usr/bin/env python

from __future__ import division
from __future__ import print_function


import sys
import select
import termios
import tty
import pdb
import json

# To get editor opening for editing:
import sys
import tempfile
import os
from subprocess import call

# for csv with string:
import csv
import io

import rospy
import actionlib
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from flo_humanoid.msg import JointTarget

from flo_core.srv import GetPoseID
from flo_core.srv import SetPose, SetPoseRequest
from flo_core.srv import SearchPose
from flo_core.srv import SetPoseSeq
from flo_core.srv import GetPoseSeqID
from flo_core.srv import SearchPoseSeq
from flo_core.msg import Pose

from flo_humanoid.msg import MoveAction, MoveGoal, MoveResult, MoveFeedback

EDITOR = os.environ.get('EDITOR', 'vim')


class Programmer(object):

    def __init__(self):
        self.commands = {'r': ('record a pose on the right arm', self.record_pose, 'right'),
                         'l': ('record a pose on the left arm', self.record_pose, 'left'),
                         'q': ('quit the program', self.quit, None),
                         'h': ('print help message', self.help, None),
                         'e': ('move to a pose on the right arm', self.move, 'right'),
                         'o': ('move to a pose on the left arm', self.move, 'left'),
                         'c': ('relax the motors', self.relax, None),
                         'p': ('search for a pose', self.search_poses, None),
                         's': ('record a motion sequence', self.record_motion_seq, None),
                         'd': ('search motion sequences', self.search_motion_seq, None),
                         'a': ('record an action sequence', self.record_action_seq, None),
                         'z': ('search action sequences', self.search_action_seq, None),
                         }

        rospy.init_node('motion_seq_programmer')
        self.run = True

        rospy.Subscriber('joint_states', JointState, self.new_joint_data)
        self.current_joint_data = None

        self.movement_client = actionlib.SimpleActionClient('move', MoveAction)
        self.movement_client.wait_for_server()

        rospy.wait_for_service('get_pose_id')
        self.get_pose_id_srv = rospy.ServiceProxy('get_pose_id', GetPoseID)
        rospy.wait_for_service('set_pose')
        self.set_pose_srv = rospy.ServiceProxy('set_pose', SetPose)
        rospy.wait_for_service('search_pose')
        self.search_pose_srv = rospy.ServiceProxy('search_pose', SearchPose)
        rospy.wait_for_service('set_pose_seq')
        self.set_pose_seq_srv = rospy.ServiceProxy('set_pose_seq', SetPoseSeq)
        rospy.wait_for_service('get_pose_seq_id')
        self.get_pose_seq_id_srv = rospy.ServiceProxy(
            'get_pose_seq_id', GetPoseSeqID)
        rospy.wait_for_service('search_pose_seq')
        self.search_pose_seq_srv = rospy.ServiceProxy(
            'search_pose_seq', SearchPoseSeq)

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
            pose_msg = Pose()
            pose_msg.description = description
            pose_msg.joint_names = clean_names
            pose_msg.joint_positions = pos
            set_pose_req = SetPoseRequest()
            set_pose_req.pose = pose_msg
            try:
                saved_id = self.set_pose_srv(set_pose_req)
                print('saved with ID: {}'.format(saved_id))
            except rospy.ServiceException as err:
                print('there was an error saving: {}'.format(err))

            tty.setcbreak(sys.stdin.fileno())
        else:
            print('no joint data available')

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
        resp = self.get_pose_id_srv(int(description))
        msg = JointTarget()
        msg.name = [side+'_'+el for el in resp.pose.joint_names]
        msg.position = resp.pose.joint_positions
        msg.target_completion_time = 2
        tty.setcbreak(sys.stdin.fileno())

        goal = MoveGoal()
        goal.targets = [msg]
        self.movement_client.send_goal(
            goal, done_cb=self.move_done, feedback_cb=self.move_feedback)

    def move_done(self, final_state, result):
        if final_state == 3:
            print('done, with positional error: {}'.format(
                result.positional_error))
        elif final_state == 2:
            print('pre-empted')
        elif final_state == 4:
            print('aborted')

    def move_feedback(self, feedback):
        print('feedback from movement: {}[elapsed] : {}[remaining] : {}[move]'.format(
            feedback.time_elapsed, feedback.time_remaining, feedback.move_number))

    def relax(self):
        self.control_pub.publish('relax')
        print('relaxing motors')

    def search_poses(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)
        target = raw_input(
            "Input the desired search term, to list all enter blank:\n")
        tty.setcbreak(sys.stdin.fileno())

        results = self.search_pose_srv(target)
        if results.ids:
            for id, desc in zip(results.ids, results.poses.description):
                print('{}: {}'.format(id, desc))
        else:
            print('no records found')

    def record_motion_seq(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)
        new_old = raw_input(
            "would you like to record a new (n) or overwrite (o) an existing sequence, cancel to cancel\n")

        if new_old == 'cancel':
            tty.setcbreak(sys.stdin.fileno())
            return
        elif new_old == 'o':
            found = False
            while not found:
                target = raw_input(
                    'what is the id of the sequence you would like to edit? cancel to cancel\n')
                if target == 'cancel':
                    tty.setcbreak(sys.stdin.fileno())
                    return
                seq = self.get_pose_seq_id_srv(int(target))
                if seq:
                    found = True
                    output_str = io.BytesIO()
                    writer = csv.writer(output_str)
                    writer.writerow('end time after prior',
                                    'pose IDs', 'arm (left, right, or both)')
                    for time, id, arm in zip(seq.times, seq.pose_ids, seq.arms):
                        writer.writerow([time, id, arm])
                    initial = output_str.getvalue()
                    prior_desc = seq.description
                else:
                    print('could not find a sequence with that ID')

        elif new_old == 'n':
            initial = 'end time after prior, pose IDs, arm (left, right, or both)'
            print('creating a new sequence')
        else:
            print('invalid option, exiting')
            return

        edited_text = self.edit_file(initial)
        # TODO: Get length

        if new_old == 'o':
            sure = raw_input(
                'you are about to overwrite sequence {}. Are you sure (yes/cancel)\n'.format(target))
            if sure == 'cancel':
                tty.setcbreak(sys.stdin.fileno())
                return
            desc = raw_input(
                'the current description is:\n{}\nto change enter a new one, otherwise enter blank\n'.format(prior_desc))
            if desc == '':
                desc = prior_desc
            self.db.ex(
                'replace into motion_sequences (id, commands, length, description) values (?,?,?,?)',
                target, edited_text, 0, desc)
        else:
            desc = raw_input('input description\n')
            self.db.ex(
                'insert into motion_sequences (commands, length, description) values (?,?,?)',
                edited_text, 0, desc)

        print('added in sequence')

    def record_action_seq(self):
        pass

    def search_motion_seq(self):
        pass

    def search_action_seq(self):
        pass

    def edit_file(self, initial_message):
        with tempfile.NamedTemporaryFile(suffix=".tmp") as tf:
            tf.write(initial_message)
            tf.flush()
            call([EDITOR, tf.name])

            # do the parsing with `tf` using regular File operations.
            # for instance:
            tf.seek(0)
            edited_message = tf.read()
        return edited_message


if __name__ == "__main__":
    pr = Programmer()
    pr.keyboard_interface()
