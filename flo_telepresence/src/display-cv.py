#!/usr/bin/env python
"""A module to display the robot screen using opencv"""

import rospy
import sys
from sensor_msgs.msg import Image as smImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import Queue

# Screen is 800x480


class RobotScreen(object):

    def __init__(self):
        self.window = cv2.namedWindow('remote_vid', cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(
            'remote_vid', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        self.image_queue = Queue.Queue()

        # cap = cv2.VideoCapture(0)
        rospy.init_node('robot_screen')
        rospy.loginfo('Started Robot Screen Node')

        self.bridge = CvBridge()
        # msg = rospy.wait_for_message("videofile/image_raw", smImage)
        # self.new_img(msg)
        rospy.Subscriber('/video_to_web', smImage, self.__new_img)
        # rospy.Subscriber('/remote_video', smImage, self.__new_img)
        self.__run_display()

    def __run_display(self):
        rate = rospy.Rate(120)
        img = None
        while not rospy.is_shutdown():
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
        # res_img = cv2.resize(cv_image, (800, 480))
        self.image_queue.put(cv_image)


if __name__ == '__main__':
    RobotScreen()
