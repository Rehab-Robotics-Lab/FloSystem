#!/usr/bin/env python

import numpy as np
import cv2
import tkinter as tk
from PIL import Image, ImageTk
import sys
import rospy
from sensor_msgs.msg import Image as smImage
from cv_bridge import CvBridge, CvBridgeError
import Queue

# Screen is 800x480
class RobotScreen(object):

    def __init__(self):
        self.window = tk.Tk()  #Makes main window
        self.window.overrideredirect(True)
        self.window.wm_attributes("-topmost", True)
        self.window.geometry("+00+00")
        self.display1 = tk.Label(self.window)
        self.display1.grid(row=1, column=0, padx=0, pady=0)  #Display 1

        self.image_queue = Queue.Queue()

        # cap = cv2.VideoCapture(0)
        rospy.init_node('robot_screen')
        rospy.loginfo('Started Robot Screen Node')

        self.bridge = CvBridge()
        # msg = rospy.wait_for_message("videofile/image_raw", smImage)
        # self.new_img(msg)
        rospy.Subscriber('videofile/image_raw', smImage, self.new_img)
        self.run_display()

    def run_display(self):
        r = rospy.Rate(120)
        while not rospy.is_shutdown():
            try:
                img = self.image_queue.get(False)
                self.show_frame(img)
            except Queue.Empty:
                pass
            self.window.update_idletasks()
            self.window.update()
            r.sleep()

    def new_img(self,msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg,"rgb8")
        except CvBridgeError as e:
            rospy.logerr('error converting message to cvmat: %s',e)
        self.image_queue.put(cv_image)

    def show_frame(self,cv_image):
        frame = cv2.resize(cv_image, (800,400))
        #frame = cv2.flip(frame, 1)
        # cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
        img = Image.fromarray(cv_image)
        imgtk = ImageTk.PhotoImage(master = self.display1, image=img)
        self.display1.imgtk = imgtk #Shows frame for display 1
        self.display1.configure(image=imgtk)

if __name__ == '__main__':
    RobotScreen()
