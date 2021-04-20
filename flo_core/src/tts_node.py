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

    # start listening to a response or waiting for some input to continue the
    # interaction

2. Play and forget
------------------

A user can also choose not to wait::

    rospy.init_node('tts_action_client')
    client = actionlib.SimpleActionClient('tts', SpeechAction)
    client.wait_for_server()
    goal = SpeechGoal()
    goal.text = 'Let me talk, you can to something else in the meanwhile.'
    client.send_goal(goal)

This is useful when the robot wants to do stuff while the audio is being
played. For example, a robot may start to
read some instructions and immediately get ready for any input.
"""

import json

import xml.etree.ElementTree as ET
from HTMLParser import HTMLParser
from StringIO import StringIO
import actionlib
import rospy
from tts.msg import SpeechAction, SpeechResult, SpeechFeedback
from tts.srv import Synthesizer
from sound_play.msg import SoundRequestAction
from flo_core_defs.msg import TTSState, TTSUtterances
import contextlib
import mutagen
from sound_play.msg import SoundRequestGoal
from sound_play.msg import SoundRequest


class MLStripper(HTMLParser):
    """A class to strip out tags

    taken from: https://stackoverflow.com/a/925630/5274985
    """

    def __init__(self):
        self.reset()
        self.text = StringIO()

    def handle_data(self, dat):
        """ingest data"""
        self.text.write(dat)

    def get_data(self):
        """return data"""
        return self.text.getvalue()


def strip_tags(html):
    """strip html tags"""
    stripper = MLStripper()
    stripper.feed(html)
    return stripper.get_data()


def play(filename):
    """plays the wav or ogg file using sound_play"""
    SoundClient(blocking=True).playWave(filename)


def do_synthesize(goal):
    """calls synthesizer service to do the job"""
    rospy.wait_for_service('synthesizer')
    synthesize = rospy.ServiceProxy('synthesizer', Synthesizer)
    return synthesize(goal.text, goal.metadata)


class TTSManager(object):
    """Action server for managing the tts system"""

    def __init__(self):
        rospy.init_node('tts_node')

        rospy.set_param("/flo_hum_vol", .9)
        rospy.set_param("/captions", False)

        self.done = True
        self.length = 0
        self.goal_text = ''
        self.result = None

        self.server = actionlib.SimpleActionServer(
            'tts', SpeechAction, self.do_speak, False)
        self.server.start()
        rospy.loginfo('Speech Action Server Ready')

        self.state_pub = rospy.Publisher('tts_state', TTSState, queue_size=1)
        self.utterance_pub = rospy.Publisher(
            'tts_utterances', TTSUtterances, queue_size=10)

        self.sound_client = actionlib.SimpleActionClient(
            'sound_play', SoundRequestAction
        )

        self.state_pub.publish(state=TTSState.WAITING)

        rospy.spin()

    def finish_with_result(self, res):
        """responds the client"""
        tts_server_result = SpeechResult(res)
        self.server.set_succeeded(tts_server_result)
        rospy.loginfo(tts_server_result)

    def do_speak(self, goal):
        """The action handler.

        Note that although it responds to client after the audio play is
        finished, a client can choose
        not to wait by not calling ``SimpleActionClient.waite_for_result()``.
        """

        # goal_root = ET.fromstring(goal.text)
        # goal_text = goal_root.text
        self.goal_text = strip_tags(goal.text)

        self.state_pub.publish(
            state=TTSState.SYNTHESIZING, text=self.goal_text)
        res = do_synthesize(goal)
        rospy.loginfo('synthesizer returns: %s', res)

        try:
            res = json.loads(res.result)
        except ValueError as err:
            syn = 'Expecting JSON from synthesizer but got {}'.format(
                res.result)
            rospy.logerr('%s. Exception: %s', syn, err)
            self.state_pub.publish(state=TTSState.ERROR, text=syn)
            self.finish_with_result(syn)
            return

        if 'Audio File' in res:
            audio_file = res['Audio File']
            rospy.loginfo('Will play %s', audio_file)

            mut_d = mutagen.File(audio_file)
            self.length = mut_d.info.length

            self.utterance_pub.publish(self.goal_text)
            msg = SoundRequest()
            # self.sendMsg(SoundRequest.PLAY_FILE, SoundRequest.PLAY_ONCE, sound,
            #      vol=volume, **kwargs)
            msg.sound = SoundRequest.PLAY_FILE
            msg.volume = rospy.get_param("/flo_hum_vol")
            msg.command = SoundRequest.PLAY_ONCE
            msg.arg = audio_file
            msg.arg2 = ""
            goal = SoundRequestGoal()
            goal.sound_request = msg
            self.done = False
            self.sound_client.send_goal(goal,
                                        active_cb=self.sound_received,
                                        feedback_cb=self.sound_fb,
                                        done_cb=self.sound_done)
            self.result = audio_file
            t_rate = rospy.Rate(10)
            success = True
            while not self.done:
                if self.server.is_preempt_requested():
                    self.sound_client.cancel_goal()
                    self.server.set_preempted()
                    success = False
                    break
                t_rate.sleep()
            self.state_pub.publish(state=TTSState.WAITING)
            if success:
                self.finish_with_result('completed sound play in')

    def sound_received(self):
        self.state_pub.publish(state=TTSState.PLAYING, text=self.goal_text)

    def sound_fb(self, feedback):
        percent_elapsed = feedback.stamp.to_sec()/self.length
        speech_feedback = SpeechFeedback()
        speech_feedback.data = self.goal_text[0:int(
            percent_elapsed*len(self.goal_text))]
        self.server.publish_feedback(speech_feedback)

    def sound_done(self, state, res):
        self.result
        self.done = True


if __name__ == '__main__':
    TTSManager()
