#! /usr/bin/env python
"""This module handles saving parameters from the server to the disk"""

from __future__ import print_function
import os.path
from datetime import datetime
import rospy
import rosparam


class ParamSaver(object):
    """A class to save Parameters on startup and shutdown"""

    # pylint: disable=too-few-public-methods
    # This file is designed to be run by the roslaunch system
    # Parameters are passed by the ros param system

    def __init__(self):
        rospy.init_node('param_saver')
        rospy.loginfo('started ros param saver')
        self.save_loc = rospy.get_param('~save_location')
        self.__save()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
        self.__save()

    def __save(self):
        file_name = os.path.expanduser(
            os.path.join(
                self.save_loc,
                'flo_parameters-{}.yaml'.format(
                    datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))))
        rospy.loginfo('saving ros params to %s', file_name)
        print(file_name)
        os.mknod(file_name)
        rosparam.dump_params(file_name, '/')


if __name__ == "__main__":
    ParamSaver()
