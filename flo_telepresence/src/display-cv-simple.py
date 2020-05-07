#!/usr/bin/env python
"""A module to display the robot screen using opencv"""

import rospy
from sensor_msgs.msg import Image as smImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import Queue

# Screen is 800x480

HOME_TIME = 5
UPDATE_HOME_TIME = 1


class RobotScreen(object):

    def __init__(self):
        rospy.init_node('robot_screen')
        self.window = cv2.namedWindow('remote_vid', cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(
            'remote_vid', cv2.WND_PROP_FULLSCREEN, 1)

        self.image_queue = Queue.Queue()

        rospy.loginfo('Started Robot Screen Node')

        self.bridge = CvBridge()
        rospy.Subscriber('/remote_video', smImage, self.__new_img)
        self.__run_display()

    def __run_display(self):
        rate = rospy.Rate(45)
        while not rospy.is_shutdown():
            img = None
            empty = False
            while not empty:
                try:
                    img = self.image_queue.get_nowait()
                except Queue.Empty:
                    empty = True
            if img is not None:
                cv2.imshow('remote_vid', img)
                cv2.waitKey(1)
            rate.sleep()

    def __new_img(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as err:
            rospy.logerr('error converting message to cvmat: %s', err)
            return
        self.image_queue.put(cv_image)


if __name__ == '__main__':
    RobotScreen()
