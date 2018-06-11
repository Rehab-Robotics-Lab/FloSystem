#!/usr/bin/env python

import rospy
import rospkg
from flo_face.msg import FaceState.msg
from flo_face.srv import GetFaceOptions, GetFaceOptionsResponse, 
                         SetEyeDirection, GetEyeDirectionResponse,
                         SetFace, SetFaceResponse
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
        self.options_service = rospy.Service('get_face_options', GetFaceOptions,self.get_face_options)
        mouth_keys = list(self.face_data['mouths'].keys())
        rospy.spin()
        
    def get_face_options(self, request):
        return GetFaceOptionsResponse(mouth_keys)

    
        


def __name__ == '__main__':
    FloFaceManager()