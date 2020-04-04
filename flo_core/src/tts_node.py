#!/usr/bin/env python

# Copyright (c) 2018, Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License").
# You may not use this file except in compliance with the License.
# A copy of the License is located at
#
#  http://aws.amazon.com/apache2.0
#
# or in the "license" file accompanying this file. This file is distributed
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
# express or implied. See the License for the specific language governing
# permissions and limitations under the License.

"""A very simple Action Server that does TTS.

It is a combination of a synthesizer and a player. Being an action server,
it can be used in two different manners.

1. Play and wait for it to finish
---------------------------------

A user can choose to be blocked until the audio playing is done.
This is especially useful in interactive scenarios.

Example::

    rospy.init_node('tts_action_client')
    client = actionlib.SimpleActionClient('tts', SpeechAction)
    client.wait_for_server()
    goal = SpeechGoal()
    goal.text = 'Let me ask you a question, please give me your answer.'
    client.send_goal(goal)
    client.wait_for_result()

    # start listening to a response or waiting for some input to continue the interaction

2. Play and forget
------------------

A user can also choose not to wait::

    rospy.init_node('tts_action_client')
    client = actionlib.SimpleActionClient('tts', SpeechAction)
    client.wait_for_server()
    goal = SpeechGoal()
    goal.text = 'Let me talk, you can to something else in the meanwhile.'
    client.send_goal(goal)

This is useful when the robot wants to do stuff while the audio is being played. For example, a robot may start to
read some instructions and immediately get ready for any input.
"""

import json

import actionlib
import rospy
from tts.msg import SpeechAction, SpeechResult
from tts.srv import Synthesizer
from flo_core_defs.msg import TTSState, TTSUtterances

from sound_play.libsoundplay import SoundClient

import xml.etree.ElementTree as ET


def play(filename):
    """plays the wav or ogg file using sound_play"""
    SoundClient(blocking=True).playWave(filename)


def do_synthesize(goal):
    """calls synthesizer service to do the job"""
    rospy.wait_for_service('synthesizer')
    synthesize = rospy.ServiceProxy('synthesizer', Synthesizer)
    return synthesize(goal.text, goal.metadata)


class TTSManager(object):

    def __init__(self):
        rospy.init_node('tts_node')

        self.server = actionlib.SimpleActionServer(
            'tts', SpeechAction, self.do_speak, False)
        self.server.start()
        rospy.loginfo('Speech Action Server Ready')

        self.state_pub = rospy.Publisher('tts_state', TTSState, queue_size=1)
        self.utterance_pub = rospy.Publisher(
            'tts_utterances', TTSUtterances, queue_size=10)

        self.state_pub.publish(state=TTSState.WAITING)

        rospy.spin()

    def finish_with_result(self, res):
        """responds the client"""
        tts_server_result = SpeechResult(res)
        self.server.set_succeeded(tts_server_result)
        rospy.loginfo(tts_server_result)

    def do_speak(self, goal):
        """The action handler.

        Note that although it responds to client after the audio play is finished, a client can choose
        not to wait by not calling ``SimpleActionClient.waite_for_result()``.
        """
        rospy.loginfo('speech goal: {}'.format(goal))

        goal_root = ET.fromstring(goal.text)
        goal_text = goal_root.text

        self.state_pub.publish(state=TTSState.SYNTHESIZING, text=goal_text)
        res = do_synthesize(goal)
        rospy.loginfo('synthesizer returns: {}'.format(res))

        try:
            res = json.loads(res.result)
        except Exception as err:
            syn = 'Expecting JSON from synthesizer but got {}'.format(
                res.result)
            rospy.logerr('{}. Exception: {}'.format(syn, err))
            self.state_pub.publish(state=TTSState.ERROR, text=syn)
            self.finish_with_result(syn)
            return

        result = ''

        if 'Audio File' in res:
            audio_file = res['Audio File']
            rospy.loginfo('Will play {}'.format(audio_file))

            self.state_pub.publish(state=TTSState.PLAYING, text=goal_text)
            self.utterance_pub.publish(goal_text)
            play(audio_file)
            result = audio_file

        if 'Exception' in res:
            result = '[ERROR] {}'.format(res)
            rospy.logerr(result)
            self.state_pub.publish(state=TTSState.ERROR, text=result)
            self.finish_with_result(syn)

        self.state_pub.publish(state=TTSState.WAITING)
        self.finish_with_result(result)


if __name__ == '__main__':
    TTSManager()
