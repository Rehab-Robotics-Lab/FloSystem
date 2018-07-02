#!/usr/bin/env python

import sys, select, tty, termios
import rospy
from flo_face.srv import (GetFaceOptions, GetFaceOptionsResponse, 
                          SetEyeDirection, SetEyeDirectionResponse,
                          SetFace, SetFaceResponse)

avialiable_face_commands = ['q','w','e','r','t','a','s','d','f','g','z','x',
                            'c','v','1','2','3','4','5','6','7','8','9']
eye_commands = {'i':'up', 'k':'center', 'j':'left', 'l':'right', ',':'down',
                'u':'up_left','o':'up_right','m':'down_left','.':'down_right'}

available_eye_commands = ['y','h','b','n','7','8','9']

mappings = dict() # mapping from button press to action key tuples

if __name__=='__main__':
    
# get options

# set terminal for sing line input

# wait for input