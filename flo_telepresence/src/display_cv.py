#!/usr/bin/env python
"""A module to display the robot screen using opencv"""

import os
import math
import Queue
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image as smImage
from rosbridge_msgs.msg import ConnectedClients
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospkg
from tts.msg import SpeechActionFeedback
import cv2
from system_monitor.msg import NETstats, HDDutil



# Screen is 800x480

HOME_TIME = 5
UPDATE_HOME_TIME = 1

# pylint: disable=too-many-arguments
def draw_text(img,  
              text,
              pos=(0, 0),
              font=cv2.FONT_HERSHEY_SIMPLEX,
              font_scale=1,
              font_thickness=2,
              text_color=(0, 255, 0),
              text_color_bg=(0, 0, 0),
              margin=5,
              num_lines=3
             ):
    """Draw text on an image using opencv

    Args:
        img: opencv compatible image
        text: text to write
        pos: the position to start drawing at
        font: the font to use (find here:
              https://docs.opencv.org/master/d6/d6e/group__imgproc__draw.html#ga0f9314ea6e35f99bb23f29567fc16e11)
        font_scale: scale to draw
        font_thickness: thickness of the lines in text
        text_color: text color as a tuple of (red, green, blue)
        text_color_bg: background color as a tuple of (red, green, blue)
        margin: the margin around the text
    """

    if text == '':
        return 0
    # pylint: disable=invalid-name
    x, y = pos 
    x = int(math.ceil(x))
    y = int(math.ceil(y))
    next_line_start = 0
    next_line_end = 0
    text_size = (0, 0)
    lines = []
    words = text.split()
    while True:
        text_size, _ = cv2.getTextSize(
            ' '.join(words[next_line_start:next_line_end+1]), font, font_scale, font_thickness)
        text_w, text_h = text_size
        if (x+margin+text_w) > img.shape[1]:
            lines.append((next_line_start, next_line_end-1))
            next_line_start = next_line_end
            next_line_end = next_line_start
        elif next_line_end+1 == len(words):
            lines.append((next_line_start, next_line_end))
            break
        else:
            next_line_end += 1
    start_y = int(math.ceil(y))
    for line in lines[max(0, len(lines)-num_lines):]:
        string_to_put = ' '.join(words[line[0]:line[1]+1])
        text_size, _ = cv2.getTextSize(
            string_to_put, font, font_scale, font_thickness)
        text_w, text_h = text_size
        cv2.rectangle(
            img,
            (x, start_y),
            (int(math.ceil(x + text_w + 2 * margin)),
             int(math.ceil(start_y + text_h + 2 * margin))),
            text_color_bg,
            -1
        )
        cv2.putText(
            img,
            string_to_put,
            (x+margin, int(text_h+margin+start_y)),
            font,
            font_scale,
            text_color,
            font_thickness
        )
        start_y = int(math.ceil(start_y + 2*margin + text_h))
    return start_y


class RobotScreen(object):
    """Class to display the screen on the robot"""

    # This is a ros node, no need for public methods:
    # pylint: disable=too-few-public-methods

    # Such is the nature of GUIs:
    # pylint: disable=too-many-instance-attributes

    def __init__(self):
        rospy.init_node('robot_screen')
        self.sim = rospy.get_param("simulate", False)
        if not self.sim:
            self.window = cv2.namedWindow('remote_vid', cv2.WINDOW_NORMAL)
            cv2.setWindowProperty(
                'remote_vid', cv2.WND_PROP_FULLSCREEN, 1)
        else:
            self.window = cv2.namedWindow('remote_vid')

        rospack = rospkg.RosPack()

        self.server_addr = os.environ['FLO_SERVER_IP']
        self.name = os.environ['ROBOT_NAME']

        self.ip_addr = ''
        self.ssid = ''
        self.wifi_quality = 0
        self.wifi_signal = 0
        self.connected_clients = 0
        self.caption = ''
        self.caption_time = 0
        self.hdd_free = 0
        self.recording = False

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
        rospy.Subscriber('/hdd_stats', HDDutil, self.__new_hdd_stats)
        rospy.Subscriber('/connected_clients', ConnectedClients,
                         self.__new_connected_clients)
        rospy.Subscriber('/record_video_status', Bool,
                         self.__set_recording_state)
        # rospy.Subscriber('/remote_video', smImage, self.__new_img)
        rospy.Subscriber('/tts/feedback', SpeechActionFeedback,
                         self.__new_caption)
        self.__run_display()

    def __new_caption(self, msg):
        self.caption = msg.feedback.data
        self.caption_time = rospy.get_time()

    def __set_recording_state(self, msg):
        self.recording = msg.data

    def __new_connected_clients(self, msg):
        self.connected_clients = len(msg.clients)

    def __new_net_stats(self, msg):
        self.ip_addr = msg.ip_addr
        self.ssid = msg.network_ssid
        self.wifi_quality = msg.link_quality
        self.wifi_signal = msg.signal_strength

    def __new_hdd_stats(self, msg):
        self.hdd_free = msg.percent_free

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
                #print(img.shape)
                rec_y = int(.0125*img.shape[0])
                bottom_rec_text = draw_text(
                    img,
                    'recording' if self.recording else 'not recording',
                    (5, rec_y),
                    self.font,
                    .00125*img.shape[0],
                    1,
                    (0, 0, 255) if self.recording else (200, 200, 0)
                )
                if rospy.get_param('/captions') and rospy.get_time() - self.caption_time < 5:
                    draw_text(
                        img,
                        self.caption,
                        (5, math.ceil(bottom_rec_text+5)),
                        self.font,
                        .002*img.shape[0],
                        1,
                        (255, 255, 255)
                    )

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
                        (10, 323),
                        self.font,
                        1,
                        (255, 255, 255))
            cv2.putText(self.filled_home,
                        'WiFi Signal: {:.1f} dB'.format(self.wifi_signal),
                        (400, 323),
                        self.font,
                        1,
                        (255, 255, 255))
            cv2.putText(self.filled_home,
                        'Server: '+self.server_addr,
                        (10, 368),
                        self.font,
                        1,
                        (255, 255, 255))
            connected = self.connected_clients > 0
            cv2.putText(self.filled_home,
                        'Connected' if connected else 'Not Connected',
                        (400, 368),
                        self.font,
                        1,
                        (0, 255, 0) if connected else (0, 0, 255))
            cv2.putText(self.filled_home,
                        'Recording' if self.recording else 'Not Recording',
                        (10, 413),
                        self.font,
                        1,
                        (0, 0, 255) if connected else (0, 255, 0))

            cv2.putText(self.filled_home,
                        'Storage free: {:.1f}%'.format(self.hdd_free),
                        (400, 413),
                        self.font,
                        1,
                        (255, 255, 255))
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
