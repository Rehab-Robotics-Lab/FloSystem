#!/usr/bin/env python

from __future__ import division
from __future__ import print_function

import math
import os
import Queue
import rospy
import rospkg
import serial
import struct
import time

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from flo_humanoid.msg import JointTarget
from read_from_bolide import BolideReader


class BolideController(object):

    CMD_version_read = 0x00
    CMD_init_motor = 0x01
    CMD_set_motor = 0x02
    CMD_capture_pos = 0x03
    CMD_relax_motor = 0x04
    CMD_SN_read = 0x05
    CMD_capture_current = 0x06
    CMD_capture_torque = 0x07
    CMD_capture_battery = 0x08
    CMD_SEQ_load_PoseCnt = 0x10
    CMD_SEQ_load_Pose = 0x11
    CMD_SEQ_load_SEQCnt = 0x12
    CMD_SEQ_load_SEQ = 0x13
    CMD_SEQ_loop_load_PoseCnt = 0x14
    CMD_SEQ_relax = 0x20
    CMD_SEQ_halt = 0x30

    NUM_MOTORS = 18

    def __init__(self):
        rospy.init_node('robot_manager')

        rospack = rospkg.RosPack()

        port = rospy.get_param('robot_port', '/dev/bolide')
        self.ser = None
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.05)
        except serial.SerialException as err:
            rospy.logerr(
                'failed to connect to bolide with err: %s', err)
            return
        rospy.loginfo('connected to robot')

        self.joint_publisher = rospy.Publisher(
            'joint_states', JointState, queue_size=1)

        rospy.Subscriber(
            'target_joint_states', JointTarget, self.new_joint_command)

        rospy.Subscriber(
            'motor_commands', String, self.new_control_command)

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
        self.joint_tasks = Queue.Queue()
        self.command_tasks = Queue.Queue()
        self.current_positions = None
        self.motors_initialized = False
        self.read_loop()

    def read_loop(self):
        """If there are motion tasks to do, do those, otherwise, check the
        robot's pose"""
        while not rospy.is_shutdown():
            while not self.command_tasks.empty():
                msg = self.command_tasks.get()
                command = msg.data
                if command == 'relax':
                    self.relax_motors()
                elif command == 'stop_motion':
                    self.upload_sequence([self.current_positions], [0])
                elif command == 'clear_poses':
                    while not self.joint_tasks.empty():
                        self.joint_tasks.get()
                elif command == 'move':
                    moves = []
                    while not self.joint_tasks.empty():
                        moves.append(self.joint_tasks.get())
                    # build unique times, for now, lets work linearly:
                    unique_times = [0]
                    for move in moves:
                        tct = move.target_completion_time
                        if tct not in unique_times:
                            unique_times.append(tct)
                    # now we need to fill in the poses:
                    poses = [list(self.current_positions)
                             for n in range(len(unique_times))]
                    # poses[0] = self.current_positions
                    current_move_program_id = [0]*self.NUM_MOTORS
                    for move in moves:
                        end_time = move.target_completion_time
                        end_id = unique_times.index(end_time)
                        for command_idx, name in enumerate(move.name):
                            motor_id = int(self.joint_config[name]['address'])
                            target_position = move.position[command_idx]
                            start_id = current_move_program_id[motor_id]
                            start_time = unique_times[start_id]
                            total_time = (end_time-start_time)
                            percents = [(ut - start_time)/total_time for
                                        ut in unique_times]
                            prior_position = poses[start_id][motor_id]
                            raw_target = target_position * int(self.joint_config[motor_id]['inversion'])*1023/(
                                2*math.pi)+int(self.joint_config[motor_id]['neutral'])
                            for motion_idx in range(start_id+1, 1+end_id):
                                # import pdb
                                # pdb.set_trace()
                                next_pose = int(round((raw_target - prior_position)
                                                      * percents[motion_idx] + prior_position))
                                poses[motion_idx][motor_id] = next_pose
                                import pdb
                                if next_pose < 0:
                                    pdb.set_trace()
                            current_move_program_id[motor_id] = end_id
                    poses = poses[1:]
                    unique_times = unique_times[1:]
                    rospy.loginfo(
                        'telling robot to go to: \n%s \nat times: \n%s', poses, unique_times)
                    self.upload_sequence(poses, unique_times)
            self.get_pose()
            self.rate.sleep()

    def get_pose(self):
        """get the pose of the robot and publish it to the joint state"""
        position = self.reader.read_data('pos')
        if not position:
            rospy.logerr('couldn\'t get postion data')
            return
        rospy.loginfo('raw position data: %s', position)
        names = []
        positions = []
        for id in self.available_motor_ids:
            raw_position = position[id]
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
        self.current_positions = position

    def new_joint_command(self, msg):
        """take a new message from the joint command topic and add it to the task queue

        :param msg: the message that is being passed in that we should parse
        """
        self.joint_tasks.put(msg)

    def new_control_command(self, msg):
        """take a new control message and add it to the control queue

        :param msg: the message being given
        """
        self.command_tasks.put(msg)

    def send_packet(self, command):
        """send_packet to the robot, add in the packet header and footer.

        :param command: the command to send, this should be given as a list
        """
        to_send = bytearray([0xff, len(command)+3]+command+[0xfe])
        rospy.loginfo('sending: %s', [hex(s) for s in to_send])
        self.ser.write(to_send)
        # feedback = br.read_feedback()
        # print(feedback)
        # time.sleep(.1)
        # print('received: {}'.format([ord(r) for r in ser.read_all()]))
        # ret = ser.read(ser.in_waiting)
        # print('sent: {}'.format(command))
        # if ret:
        #     print('received: {}'.format([ord(c) for c in ret]))
        # todo: really should have feedback checking see bolide-...0.ino:378

    def relax_motors(self):
        """relax_motors"""
        self.send_packet([self.CMD_SEQ_relax])
        self.motors_initialized = False

    def upload_pose(self, id, pose):
        """upload_pose, send the given pose to the robot to prepare for motion

        :param id: The number of the pose. Supported values are 0-255
        :param pose: The actual Pose, an 18 element array
        """
        command = [None] * (self.NUM_MOTORS*2 + 2)
        command[0] = self.CMD_SEQ_load_Pose
        command[1] = id
        for idx, motor in enumerate(pose):
            bb, lb = struct.pack('>H', motor)
            bb = (ord(bb))
            lb = (ord(lb))
            command[idx*2+2] = bb
            command[idx*2+3] = lb
        self.send_packet(command)

    def initialize_motors(self):
        """initialize_motors on the robot to prepare for motion.
        Must be run before trying to move"""
        self.send_packet([self.CMD_init_motor, self.NUM_MOTORS])

    def upload_poses(self, poses):
        """upload_poses to the robot for a motion sequence.

        :param poses: The poses to upload. This should be a list of lists,
                      with each inner list of length number of motors.
        """
        self.send_packet([self.CMD_SEQ_load_PoseCnt, len(poses)])
        for idx, pose in enumerate(poses):
            self.upload_pose(idx, pose)

    def upload_sequence(self, poses, times):
        """upload_sequence, uploads the entire set of poses to the robot with
           the times that the poses are supposed to be hit. Once this command
           finishes, the robot will begin moving.

        :param poses: The poses which you want to pass through
        :param times: The times which you want to hit each pose. Note that the
                      space between each pose will be hit in linear fashion.
        """
        if not self.motors_initialized:
            self.initialize_motors()
        self.upload_poses(poses)
        self.send_packet([self.CMD_SEQ_load_SEQCnt, len(times)])
        for idx, time in enumerate(times):
            # import pdb
            # pdb.set_trace()
            # time is in units of 10ms on the robot. But in sec coming in
            # TODO: somewhere there should be a check to make sure time is <10 sec
            ms_time = time*1000
            bb, lb = struct.pack('>H', ms_time)
            bb = (ord(bb))
            lb = (ord(lb))
            self.send_packet([self.CMD_SEQ_load_SEQ, idx, bb, lb])


if __name__ == "__main__":
    CONTROLLER = BolideController()
