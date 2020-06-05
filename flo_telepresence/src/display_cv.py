#!/usr/bin/env python
"""A module to display the robot screen using opencv"""

import os
import Queue
import rospy
from sensor_msgs.msg import Image as smImage
from rosbridge_msgs.msg import ConnectedClients
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospkg
import cv2
from system_monitor.msg import NETstats

# Screen is 800x480

HOME_TIME = 5
UPDATE_HOME_TIME = 1


class RobotScreen(object):
    """Class to display the screen on the robot"""

    # This is a ros node, no need for public methods:
    # pylint: disable=too-few-public-methods

    # Such is the nature of GUIs:
    # pylint: disable=too-many-instance-attributes

    def __init__(self):
        rospy.init_node('robot_screen')
        self.window = cv2.namedWindow('remote_vid', cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(
            'remote_vid', cv2.WND_PROP_FULLSCREEN, 1)

        rospack = rospkg.RosPack()

        self.server_addr = os.environ['FLO_SERVER_IP']
        self.name = os.environ['ROBOT_NAME']

        self.ip_addr = ''
        self.ssid = ''
        self.wifi_quality = 0
        self.wifi_signal = 0
        self.connected_clients = 0

        self.image_queue = Queue.Queue()

        self.last_msg = 0
        self.last_home_update = 0

        self.home_screen = np.zeros((480, 800, 3), np.uint8)
        logo = cv2.imread(
            os.path.join(
                rospack.get_path('flo_telepresence'), 'RRLlogo.png'),
            cv2.IMREAD_UNCHANGED
        )
        small_logo = cv2.resize(logo, (140, 140))
        clean_logo = self.home_screen[10:150, 10:150]
        where_logo = np.where(small_logo[:, :, 3] == 255)
        color_logo = small_logo[:, :, 0:3]
        clean_logo[where_logo] = color_logo[where_logo]
        self.home_screen[10:150, 10:150] = clean_logo
        self.font = cv2.FONT_HERSHEY_TRIPLEX
        cv2.putText(self.home_screen,
                    'Flo Robots by RRL',
                    (170, 100),
                    self.font,
                    1.8,
                    (255, 255, 255))
        cv2.putText(self.home_screen,
                    'Name: '+self.name,
                    (10, 214),
                    self.font,
                    1,
                    (255, 255, 255))
        cv2.putText(self.home_screen,
                    'For Help: mjsobrep@seas.upenn.edu (770)324-6196',
                    (10, 470),
                    self.font,
                    .8,
                    (255, 255, 255))
        self.filled_home = self.home_screen.copy()

        # cap = cv2.VideoCapture(0)
        rospy.loginfo('Started Robot Screen Node')

        self.bridge = CvBridge()
        # msg = rospy.wait_for_message("videofile/image_raw", smImage)
        # self.new_img(msg)
        rospy.Subscriber('/remote_video_clean', smImage, self.__new_img)
        rospy.Subscriber('/net_stats', NETstats, self.__new_net_stats)
        rospy.Subscriber('/connected_clients', ConnectedClients,
                         self.__new_connected_clients)
        # rospy.Subscriber('/remote_video', smImage, self.__new_img)
        self.__run_display()

    def __new_connected_clients(self, msg):
        self.connected_clients = len(msg.clients)

    def __new_net_stats(self, msg):
        self.ip_addr = msg.ip_addr
        self.ssid = msg.network_ssid
        self.wifi_quality = msg.link_quality
        self.wifi_signal = msg.signal_strength

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
            elif rospy.get_time()-self.last_msg > HOME_TIME:
                self.__draw_home()
            rate.sleep()

    def __draw_home(self):
        if rospy.get_time()-self.last_home_update > UPDATE_HOME_TIME:
            self.filled_home = self.home_screen.copy()
            cv2.putText(self.filled_home,
                        'IP: '+self.ip_addr,
                        (400, 278),
                        self.font,
                        1,
                        (255, 255, 255))
            cv2.putText(self.filled_home,
                        'SSID: '+self.ssid,
                        (10, 278),
                        self.font,
                        1,
                        (255, 255, 255))
            cv2.putText(self.filled_home,
                        'WiFi Quality: {:.1f}'.format(self.wifi_quality),
                        (10, 342),
                        self.font,
                        1,
                        (255, 255, 255))
            cv2.putText(self.filled_home,
                        'WiFi Signal: {:.1f} dB'.format(self.wifi_signal),
                        (400, 342),
                        self.font,
                        1,
                        (255, 255, 255))
            cv2.putText(self.filled_home,
                        'Server: '+self.server_addr,
                        (10, 406),
                        self.font,
                        1,
                        (255, 255, 255))
            connected = self.connected_clients > 0
            cv2.putText(self.filled_home,
                        'Connected' if connected else 'Not Connected',
                        (400, 406),
                        self.font,
                        1,
                        (0, 255, 0) if connected else (0, 0, 255))
            cv2.imshow('remote_vid', self.filled_home)
            cv2.waitKey(1)
            self.last_home_update = rospy.get_time()

    def __new_img(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as err:
            rospy.logerr('error converting message to cvmat: %s', err)
            return
        # res_img = cv2.resize(cv_image, (800, 480))
        self.image_queue.put(cv_image)
        self.last_msg = rospy.get_time()


if __name__ == '__main__':
    RobotScreen()
