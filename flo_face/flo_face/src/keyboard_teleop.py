#!/usr/bin/env python
# pylint: skip-file

import sys
import select
import tty
import termios
import rospy
from flo_face.srv import (GetFaceOptions, GetFaceOptionsResponse,
                          SetEyeDirection, SetEyeDirectionResponse,
                          SetFace, SetFaceResponse)

avialiable_commands = ['q', 'w', 'e', 'r', 't', 'a', 's', 'd', 'f', 'g', 'z', 'x',
                            'c', 'v', '1', '2', '3', '4', '5', '6', 'y', 'h', 'b', 'n', '7',
                            '8', '9']
clean_eye_commands = {'i': 'up', 'k': 'center', 'j': 'left', 'l': 'right', ',': 'down',
                      'u': 'up_left', 'o': 'up_right', 'm': 'down_left', '.': 'down_right',
                      '0': 'default'}


mappings = dict()  # mapping from button press to action key tuples
funky_eyes = dict()
eye_commands = dict()
eye_commands.update(clean_eye_commands)
eye_commands.update(funky_eyes)
if __name__ == '__main__':
    rospy.init_node('face_keyboard_teleop')
    # get services
    rospy.wait_for_service('get_face_options')
    get_options = rospy.ServiceProxy('get_face_options', GetFaceOptions)
    rospy.wait_for_service('set_eye_direction')
    set_eye_direction = rospy.ServiceProxy(
        'set_eye_direction', SetEyeDirection)
    rospy.wait_for_service('set_face')
    set_face = rospy.ServiceProxy('set_face', SetFace)
    # get options
    face_options = get_options()
    print('Available Faces:')
    print('key\tface')
    for idx, face in enumerate(face_options.faces):
        key = avialiable_commands.pop(0)
        mappings[key] = face
        print('{}\t{}'.format(key, face))
    print('to change eye direction, your options are:\nkey\tdirection')
    for key, direction in eye_commands.items():
        print('{}\t{}'.format(key, direction))
    print('you can press 0 to activate the default eyes')
    # set terminal for single line input
    rate = rospy.Rate(100)
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print('ready for input. Press Ctrl-C to exit')
    # wait for input
    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            new_input = sys.stdin.read(1)
            if new_input in mappings:
                desired_face = mappings[new_input]
                resp = set_face(desired_face)
                if resp.success:
                    avialiable_commands.extend(funky_eyes.keys())
                    funky_eyes = dict()
                    print('changed face to {} with info: {}'.format(
                        desired_face, resp.info))
                    print('Available Eye Directions: {}'.format(
                        resp.available_eye_directions))
                    for direction in resp.available_eye_directions:
                        if direction not in eye_commands.values():
                            key = avialiable_commands.pop(0)
                            funky_eyes[key] = direction
                            print('you can press {} to activate the {} eyes'.format(
                                key, direction))
                    eye_commands = dict()
                    eye_commands.update(clean_eye_commands)
                    eye_commands.update(funky_eyes)
                else:
                    print('failed to set new face with info: {}'.format(
                        resp.info))
            elif new_input in eye_commands:
                resp = set_eye_direction(eye_commands[new_input])
                if resp.success:
                    desired_direction = eye_commands[new_input]
                    print('changed eye direction to {} with info: {}'.format(
                        desired_direction, resp.info))
                else:
                    print('failed to change eye direction with info: {}'.format(
                        resp.info))

        rate.sleep()
    # return to default state
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
