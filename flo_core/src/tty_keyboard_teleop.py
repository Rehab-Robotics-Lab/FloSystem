#!/usr/bin/env python

import rospy
import actionlib
from tts.msg import SpeechGoal, SpeechAction
import json

if __name__=='__main__':
    rospy.init_node('tts_keyboard_teleop')
    rospy.loginfo('Node Up')
    client = actionlib.SimpleActionClient('tts', SpeechAction)
    client.wait_for_server()

    rospy.loginfo('TTS Available')

    while not rospy.is_shutdown():
        to_say = raw_input("enter something to say:")
        goal = SpeechGoal()
        goal.text = str(to_say)
        goal.metadata = json.dumps({"voice_id": "Ivy"}) 
        # could also include parameters here: https://github.com/aws-robotics/tts-ros1/blob/23e7aa554e7c4717de15fbf6c28dd090b3cb89df/tts/src/tts/synthesizer.py#L131
        client.send_goal(goal)
        rospy.loginfo('sent command to robot')
        client.wait_for_result()
        rospy.loginfo('done speaking')

