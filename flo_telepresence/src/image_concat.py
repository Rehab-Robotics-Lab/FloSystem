#!/usr/bin/env python
"""A module for concatanating, scaling, and throtling the video from the various feeds"""

import rospy
import sys
from sensor_msgs.msg import Image as smImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import threading

# Screen is 800x480


class RingBuffer(object):

    def __init__(self, size):
        self.size = size
        self.list = [None]*size
        self.latest = self.size - 1
        self.safe = [True]*size
        self.lock = threading.Lock()
        self.initialized = False
        self.last = 0
        self.repeats = 0

    def put(self, data):
        with self.lock:
            self.latest = (self.latest+1) % self.size
            write_loc = self.latest
            if not self.safe[write_loc]:
                rospy.logerr('buffer did not have space during add')
                raise Exception('Buffer did not have space durring add')
            self.safe[write_loc] = False
        self.list[write_loc] = data
        with self.lock:
            self.safe[write_loc] = True
            self.initialized = True

    def get_latest(self):
        if not self.initialized:
            rospy.logerr('no data yet exists')
            raise Exception('No data yet exists')
        with self.lock:
            target = self.latest
            found_safe = False
            for i in range(self.size):
                safe_target = (target-i) % self.size
                if self.safe[safe_target]:
                    found_safe = True
                    self.safe[safe_target] = False
                    break
            if not found_safe:
                rospy.logerr('no safe data found')
                raise Exception('No safe data found')
        to_return = self.list[safe_target]
        with self.lock:
            self.safe[safe_target] = True
        if self.last == safe_target:
            self.repeats = self.repeats + 1
        else:
            self.repeats = 0
            self.last = safe_target
        if self.repeats > 1:
            rospy.logwarn('Image concatenation is slowing down badly')
        return to_return


class Feed(object):

    def __init__(self, topic_name):
        rospy.Subscriber(topic_name, smImage, self.__new_img)
        self.bridge = CvBridge()
        self.buffer = RingBuffer(5)

    def __new_img(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as err:
            return
        # res_img = cv2.resize(cv_image, (800, 480))
        self.buffer.put(cv_image)

    def get_latest(self):
        return self.buffer.get_latest()


class RTCPrep(object):

    def __init__(self):
        rospy.init_node('image_concat')

        subscriber_list = ['/upper_realsense/color/image_raw',
                           '/lower_realsense/color/image_raw',
                           '/fisheye_cam/image_raw']

        publisher = rospy.Publisher(
            'video_to_web', smImage, queue_size=1, latch=True)

        bridge = CvBridge()

        feeds = []
        for subscribe in subscriber_list:
            feeds.append(Feed(subscribe))

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            try:
                images = [None]*len(feeds)
                for idx, feed in enumerate(feeds):
                    images[idx] = feed.get_latest()

                publisher.publish(bridge.cv2_to_imgmsg(
                    cv2.hconcat(images), 'bgr8'))
            except:
                rospy.logerr('error while concatanating latest imges')

            rate.sleep()


if __name__ == '__main__':
    RTCPrep()
