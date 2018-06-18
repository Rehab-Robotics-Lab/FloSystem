#!/usr/bin/env python

import rospy
import rospkg
import json
from flo_face.msg import FaceState
from flo_face.srv import (GetFaceOptions, GetFaceOptionsResponse, 
                          SetEyeDirection, SetEyeDirectionResponse,
                          SetFace, SetFaceResponse)
from os.path import expanduser, join

class FloFaceManager(object):
    """handles loading face options, and providing those face options to other 
    nodes. It receives messages with names of faces to use and directions to 
    look, it then broadcasts the current desired state."""

    def __init__(self):
        rospy.init_node('face_manager')
        self.rospack = rospkg.RosPack()
        self.face_fn = expanduser(rospy.get_param('face_json',
        join(self.rospack.get_path('flo_face'),'data','faces.json')))
        with open(self.face_fn) as file:
            self.face_data = json.load(file)
        self.mouth_keys = list(self.face_data['mouths'].keys())
        self.eye_direction = 'center'
        self.current_mouth = 'standard'
        self.current_eyes = 'standard'

        self.state_pub = rospy.Publisher('face_state',FaceState,queue_size=1)
        self.set_eye_service = rospy.Service('set_eye_direction', SetEyeDirection, self.set_eye_direction)
        self.set_face_service = rospy.Service('set_face', SetFace, self.set_face)
        self.options_service = rospy.Service('get_face_options', GetFaceOptions,self.get_face_options)      
        rospy.loginfo('face manager up')  
        rospy.spin()

    def get_face_options(self, request):
        return GetFaceOptionsResponse(self.mouth_keys)  
        
    def set_face(self, request):
        flatten = lambda l: [item for sublist in l for item in sublist]
        resp = SetFaceResponse()
        new_state = FaceState()
        if request.face in self.mouth_keys:
            new_mouth = self.face_data['mouths'][request.face]
            new_state.mouth = flatten(new_mouth['on'])
            new_state.mouth_width = len(new_mouth['on'][0])
            new_state.mouth_height = len(new_mouth['on'])
            new_state.mouth_name = request.face
            new_state.mouth_description = new_mouth['description']
            self.current_mouth = request.face
            self.current_eyes = new_mouth['eyes']
            new_eye_data = self.face_data['eyes'][self.current_eyes]
            if self.eye_direction not in new_eye_data:
                self.eye_direction = new_eye_data['default']
            new_state.eye_name = self.current_eyes
            if 'left' in new_eye_data[self.eye_direction]:
                new_state.left_eye = flatten(new_eye_data[self.eye_direction]['left']['on'])
                new_state.right_eye = flatten(new_eye_data[self.eye_direction]['right']['on'])
                new_state.eye_width = len(new_eye_data[self.eye_direction]['right']['on'][0])
                new_state.eye_height = len(new_eye_data[self.eye_direction]['right']['on'])
            else:
                new_state.left_eye = flatten(new_eye_data[self.eye_direction]['on'])
                new_state.right_eye = new_state.left_eye
                new_state.eye_width = len(new_eye_data[self.eye_direction]['on'][0])
                new_state.eye_height = len(new_eye_data[self.eye_direction]['on'])            
            resp.success = True
            resp.info = 'face sent'
            resp.available_eye_directions = list(new_eye_data.keys())
            self.state_pub.publish(new_state)
        else:
            resp.success = False
            resp.info = 'invalid face string'
        return resp

    def set_eye_direction(sef, request):
        return None


if __name__ == '__main__':
    FloFaceManager()