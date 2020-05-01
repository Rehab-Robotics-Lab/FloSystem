#!/usr/bin/env python

import json
from os.path import expanduser, join
import rospy
import rospkg
from flo_face_defs.msg import FaceState
from flo_face_defs.srv import (GetFaceOptions, GetFaceOptionsResponse,
                               SetEyeDirection, SetEyeDirectionResponse,
                               SetFace, SetFaceResponse, SetFaceRequest,
                               SetFaceBrightness, SetFaceBrightnessResponse)


class FloFaceManager(object):
    """handles loading face options, and providing those face options to other
    nodes. It receives messages with names of faces to use and directions to
    look, it then broadcasts the current desired state."""

    def __init__(self):
        rospy.init_node('face_manager')
        self.rospack = rospkg.RosPack()
        self.face_fn = expanduser(
            rospy.get_param('face_json', join(
                self.rospack.get_path('flo_face'), 'data', 'faces.json')))
        with open(self.face_fn) as file:
            self.face_data = json.load(file)
        self.mouth_keys = list(self.face_data['mouths'].keys())
        self.eye_direction = 'center'
        self.current_mouth = 'standard'
        self.current_eyes = 'standard'
        self.new_state = FaceState()
        self.new_state.mouth_brightness = 12
        self.new_state.right_eye_brightness = 12
        self.new_state.left_eye_brightness = 12

        self.state_pub = rospy.Publisher(
            'face_state', FaceState, queue_size=1, latch=True)
        self.set_eye_service = rospy.Service(
            'set_eye_direction', SetEyeDirection, self.set_eye_direction)
        self.set_face_service = rospy.Service(
            'set_face', SetFace, self.set_face)
        self.options_service = rospy.Service(
            'get_face_options', GetFaceOptions, self.get_face_options)
        self.set_brightness_service = rospy.Service(
            'set_face_brightness', SetFaceBrightness, self.set_brightness)
        # self.set_face(SetFace(self.current_mouth))
        rospy.loginfo('face manager up')
        self.set_face(SetFaceRequest('sleep'))
        rospy.spin()

    def get_face_options(self, _):
        """ Get and return available faces as represented by the available
        mouths.

        Returns: The list of faces, as names
        """
        return GetFaceOptionsResponse(self.mouth_keys)

    def set_face(self, request):
        """Receive a request to set the face. Load the new mouth and
        the eyes into the new state structure and the current mouth
        and eyes. If the new face doesn't have the current eye
        type/direction, the eyes will be set to their default.
        The available eye directions are returned with other
        info and the service is returned.

        The new face stat is published

        Args:
            request: a simple service request with a single
            field `face` which is the key for the face to set.

        Returns: A service response with the available eye
        directions given the new face, whether the operation
        succeeded or not and information on what happened.
        """
        resp = SetFaceResponse()
        if request.face in self.mouth_keys:
            new_mouth = self.face_data['mouths'][request.face]
            self.new_state.mouth = self.flatten(new_mouth['on'])
            self.new_state.mouth_width = len(new_mouth['on'][0])
            self.new_state.mouth_height = len(new_mouth['on'])
            self.new_state.mouth_name = request.face
            self.new_state.mouth_description = new_mouth['description']
            self.current_mouth = request.face
            self.current_eyes = new_mouth['eyes']
            new_eye_data = self.face_data['eyes'][self.current_eyes]
            if self.eye_direction not in new_eye_data:
                self.eye_direction = new_eye_data['default']
            self.new_state.eye_name = self.current_eyes
            self.set_eye()
            resp.success = True
            resp.info = 'face sent'
            resp.available_eye_directions = list(new_eye_data.keys())
            self.state_pub.publish(self.new_state)
        else:
            resp.success = False
            resp.info = 'invalid face string'
        return resp

    @staticmethod
    def flatten(lst):
        """Flatten out a matrix as a list, unraveling it so that it
        can be sent

        Args:
            lst: The list representation of the matrix

        Returns: The flattened list
        """
        return [item for sublist in lst for item in sublist]

    def set_eye(self):
        """Sets the eyes in the new_state structure based on
        the current values for the eyes and face.
        """
        new_eye_data = self.face_data['eyes'][self.current_eyes]
        if 'left' in new_eye_data[self.eye_direction]:
            self.new_state.left_eye = self.flatten(
                new_eye_data[self.eye_direction]['left']['on'])
            self.new_state.right_eye = self.flatten(
                new_eye_data[self.eye_direction]['right']['on'])
            self.new_state.eye_width = len(
                new_eye_data[self.eye_direction]['right']['on'][0])
            self.new_state.eye_height = len(
                new_eye_data[self.eye_direction]['right']['on'])
        else:
            self.new_state.left_eye = self.flatten(
                new_eye_data[self.eye_direction]['on'])
            self.new_state.right_eye = self.new_state.left_eye
            self.new_state.eye_width = len(
                new_eye_data[self.eye_direction]['on'][0])
            self.new_state.eye_height = len(
                new_eye_data[self.eye_direction]['on'])

    def set_eye_direction(self, request):
        """Fiedld a request to set the eye direction, set it
        and return whether it succeeded.

        If the requested direction is `default` then that is
        loaded in and replaced by the actual direction which
        is the default. Once the eye direction is set, then
        the `set_eyes()` function is called to load the
        actual data.
        If the requested direction is `default` then that is
        loaded in and replaced by the actual direction which
        is the default.

        Args:
            request: a simple service request with a single
                     field `direction`, a string which
                     selects the eye direction

        Returns: A service response with whether the service
                 succeeded and info about that.
        """
        resp = SetEyeDirectionResponse()
        new_eye_data = self.face_data['eyes'][self.current_eyes]
        if request.direction in new_eye_data:
            if request.direction == 'default':
                request.direction = new_eye_data['default']
            self.eye_direction = request.direction
            self.set_eye()
            resp.success = True
            resp.info = 'all good'
            self.state_pub.publish(self.new_state)
        else:
            resp.success = False
            resp.info = 'direction cannot be set for this face'
        return resp

    def set_brightness(self, request):
        """Set the brightness of the LEDs

        Args:
            request: The service request

        Returns: The service response
        """
        resp = SetFaceBrightnessResponse()
        resp.success = True
        if request.value > 15:
            resp.success = False
            resp.info = 'value too high'
        elif request.value < 0:
            resp.success = False
            resp.info = 'value too low'
        elif request.target == 'all':
            self.new_state.right_eye_brightness = request.value
            self.new_state.left_eye_brightness = request.value
            self.new_state.mouth_brightness = request.value
        elif request.target == 'left_eye':
            self.new_state.left_eye_brightness = request.value
        elif request.target == 'right_eye':
            self.new_state.right_eye_brightness = request.value
        elif request.target == 'mouth':
            self.new_state.mouth_brightness = request.value
        else:
            resp.success = False
            resp.info = 'invalid target'

        if resp.success:
            self.state_pub.publish(self.new_state)

        return resp


if __name__ == '__main__':
    FloFaceManager()
