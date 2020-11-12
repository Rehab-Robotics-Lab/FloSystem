#! /usr/bin/env python
"""This module handles turning on and off recording"""

from __future__ import print_function
import rospy
from std_msgs.msg import Bool
from flo_core_defs.srv import SetRecording


class RecordingManager(object):
    """A class to manage whether to record video"""

    # pylint: disable=too-few-public-methods
    # This file is designed to be run by the roslaunch system
    # Parameters are passed by the ros param system

    def __init__(self):
        rospy.init_node('recording-manager')
        rospy.loginfo('started recording manager')
        self.recording = False
        self.pub_dummy = rospy.Publisher(
            '/record_video_dummy', Bool, queue_size=1)
        pub_status = rospy.Publisher(
            '/record_video_status', Bool, queue_size=1)
        rospy.Service(
            'set_recording', SetRecording, self.__serv_req)
        self.sub_dummy = None
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            subs = self.pub_dummy.get_num_connections()
            if subs > 0:
                pub_status.publish(True)
            else:
                pub_status.publish(False)
            rate.sleep()

    def __serv_req(self, req):
        self.recording = req.record
        if req.record:
            if self.sub_dummy is None:
                self.sub_dummy = rospy.Subscriber(
                    '/record_video_dummy', Bool, lambda: None)
        else:
            if self.sub_dummy is not None:
                self.sub_dummy.unregister()
                self.sub_dummy = None

        subs = self.pub_dummy.get_num_connections()
        if req.record:
            if subs > 0:
                return True
            return False
        else:
            if subs > 0:
                return False
            return True


if __name__ == "__main__":
    RecordingManager()
