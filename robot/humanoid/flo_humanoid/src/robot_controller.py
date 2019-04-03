#!/usr/bin/env python3

from read_from_bolide import BolideReader 
from sensor_msgs.msg import JointState

class BolideController:
    def __init__(self):
        rospy.init_node('robot_manager')
        
        self.port = rospy.get_param('robot_port','/dev/bolide')
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=0.2)
        except:
            rospy.logerr('failed to connect to bolide')
        rospy.loginfo('connected to robot')

        self.joint_publisher = rospy.Publisher(
                'joint_states', JointState, queue_size=1)

        self.command_reader = rospy.Subscriber('target_joint_states',JointStateTarget,self.new_command)

        self.reader = BolideReader(self.ser)
        #TODO: read in a paramater file describing servo offsets

    def get_pos(self):
        position = self.reader.read_data('pos')
        if not position:
            rospy.logerr('couldn\'t get postion data')
            return

        #TODO: take position data and turn it into something that can be broadcast

    def new_command(self, msg):
        #TODO: take in a commanded joint state and target system time to achieve it, interpolate it out to make it happen and send the command to the robot

