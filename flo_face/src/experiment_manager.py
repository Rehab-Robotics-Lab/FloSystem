#!/usr/bin/env python

import sys, select, tty, termios
import rospy
from flo_face.srv import (GetFaceOptions, GetFaceOptionsResponse, 
                          SetEyeDirection, SetEyeDirectionResponse,
                          SetFace, SetFaceResponse)
import random
import pdb
import signal

static_mode = 'happy'

dynamic_mode = ['chortling','cheeky','content','laughing','smiling_hard','happy']

def change_face(name):
    resp = set_face(name)
    if not resp.success:
        rospy.logerr('couldn\'t get the face changed: {} when using key: {}'.format(resp.info,name))
    elif resp.success:
        rospy.loginfo('set face to:\t{}'.format(name))
    return resp.available_eye_directions

def calc_next_face_change():
    return rospy.get_rostime() + rospy.Duration(random.uniform(7,15))

def calc_next_eye_change():
    return rospy.get_rostime() + rospy.Duration(random.uniform(3,10))

mode = 'static'
if __name__=='__main__':
    rospy.init_node('face_exp_controller')
    # get services
    rospy.wait_for_service('get_face_options')
    get_options = rospy.ServiceProxy('get_face_options', GetFaceOptions)
    rospy.wait_for_service('set_eye_direction')
    set_eye_direction = rospy.ServiceProxy('set_eye_direction', SetEyeDirection)
    rospy.wait_for_service('set_face')
    set_face = rospy.ServiceProxy('set_face', SetFace)
    # get options
    face_options = get_options()
    all_faces = face_options.faces
    # set default
    available_eyes = change_face(static_mode)
    next_eye_change = calc_next_eye_change()
    # set terminal for single line input
    rate = rospy.Rate(100)
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print('ready for input. Press Ctrl-C to exit')
    print('press s to enable static mode, d to enable dynamic mode, f to enable the full cycle mode')
    def sigint_handler(signum, frame):
        print('exiting')
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
        sys.exit()
    signal.signal(signal.SIGINT, sigint_handler)

    # wait for input
    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            new_input = sys.stdin.read(1)
            if new_input == 's':
                mode = 'static'
                available_eyes = change_face(static_mode)
                set_eye_direction('default')
                print('set to static mode')
            elif new_input == 'd':
                mode = 'dynamic'
                next_face_change = calc_next_face_change()
                print('set to dynamic mode')
            elif new_input == 'f':
                mode = 'cycling'
                random.shuffle(all_faces)
                cycle_idx = 0
                print('Set to cycle through all faces, press the n key to go to the next face')
            elif new_input == 'n' and mode == 'cycling':
                available_eyes = change_face(all_faces[cycle_idx])
                cycle_idx = (cycle_idx + 1)
                if cycle_idx >= len(all_faces):
                    print('All faces cycled through')
                    mode = 'static'
                    available_eyes = change_face(static_mode)
                    print('set back to static')
            
        if (mode == 'cycling' or mode == 'dynamic') and rospy.get_rostime() > next_eye_change:
            resp = set_eye_direction(random.choice(available_eyes))
            next_eye_change = calc_next_eye_change()
        if  mode == 'dynamic' and rospy.get_rostime() > next_face_change:
            available_eyes = change_face(random.choice(dynamic_mode))
            next_face_change = calc_next_face_change()
        rate.sleep()
    # return to default state
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)