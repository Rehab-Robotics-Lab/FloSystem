#!/usr/bin/env python
"""A module for displaying the robot screen using a tkinter canvas"""

import sys
import rospy
from PIL import Image, ImageTk
from sensor_msgs.msg import Image as smImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk
import Queue

# Screen is 800x480


class RobotScreen(object):

    def __init__(self):
        self.window = tk.Tk()  # Makes main window
        self.window.overrideredirect(True)
        self.window.wm_attributes("-topmost", True)
        self.window.geometry("800x480+0+0")
        self.canvas = tk.Canvas(self.window, width=800, height=480)
        self.canvas.grid(row=0, column=0, padx=0, pady=0)  # Display 1
        self.image_on_canvas = None

        self.image_queue = Queue.Queue()

        # cap = cv2.VideoCapture(0)
        rospy.init_node('robot_screen')
        rospy.loginfo('Started Robot Screen Node')

        self.bridge = CvBridge()
        # msg = rospy.wait_for_message("videofile/image_raw", smImage)
        # self.new_img(msg)
        rospy.Subscriber('/remote_video', smImage, self.__new_img)
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
                self.__show_frame(img)
                self.window.update_idletasks()
                self.window.update()
            rate.sleep()

    def __new_img(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except CvBridgeError as err:
            rospy.logerr('error converting message to cvmat: %s', err)
            return
        res_img = cv2.resize(cv_image, (800, 480))
        self.image_queue.put(res_img)

    def __show_frame(self, cv_image):
        #frame = cv2.flip(frame, 1)
        # cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
        img = Image.fromarray(cv_image)
        imgtk = ImageTk.PhotoImage(master=self.canvas, image=img)
        self.canvas.imgtk = imgtk
        if self.image_on_canvas is None:
            self.image_on_canvas = self.canvas.create_image(
                0, 0, anchor=tk.NW, image=imgtk)
            print('setup first image')
        else:
            self.canvas.itemconfig(self.image_on_canvas, image=imgtk)

        # self.display1.imgtk = imgtk  # Shows frame for display 1
        # This is a very very slow operation:
        # self.display1.configure(image=imgtk)


if __name__ == '__main__':
    RobotScreen()
