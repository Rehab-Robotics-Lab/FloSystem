#! /usr/bin/env python

import os.path
from datetime import datetime
import rospy
import rosparam


class ParamSaver(object):
    def __init__(self):
        rospy.init_node('param_saver')
        rospy.loginfo('started ros param saver')
        self.save_loc = rospy.get_param('~save_location')
        self.save()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
        self.save()

    def save(self):
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
