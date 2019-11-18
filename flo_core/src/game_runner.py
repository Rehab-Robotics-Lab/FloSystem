#!/usr/bin/env python

from enum import Enum
import json
import random
import queue
import rospy
import actionlib
from tts.msg import SpeechAction, SpeechGoal
from flo_humanoid.msg import MoveAction, MoveGoal, JointTarget
from flo_core.msg import GameFeedback, GameCommandOptions, GameDef, GameCommand


class GameRunner(object):
    states = Enum(
        'states', 'waiting_for_def game_loaded acting waiting_for_command')

    def __init__(self):
        rospy.init_node('game_runner')

        ### Action Servers to Make Things Happen ###
        # set up polly action server
        self.speech_server = actionlib.SimpleActionClient(
            'tts', SpeechAction)
        self.speech_server.wait_for_server()

        # setup movement action server
        self.move_server = actionlib.SimpleActionClient('move', MoveAction)
        self.move_server.wait_for_server()

        ### Publishers ###
        self.feedback_pub = rospy.Publisher('game_runner_feedback',
                                            GameFeedback, queue_size=1, latch=True)
        self.command_opts_pub = rospy.Publisher('game_runner_command_opts',
                                                GameCommandOptions, queue_size=1, latch=True)

        ### Subscribers ###
        self.command_sub = rospy.Subscriber(
            'game_runner_commands', GameCommand, self.new_command)
        self.definition_sub = rospy.Subscriber(
            'game_def', GameDef, self.new_def)

        ### Queue's for safe receiving of info ###
        self.command_queue = queue.Queue()
        self.def_queue = queue.Queue()

        ### State Management ###
        self.state = self.states.waiting_for_def
        self.moving_state = 'none'
        self.speaking_state = 'none'

        ### The actions that compose this game ###
        self.actions_list = []
        self.action_idx = -1
        self.command_opts = []

        ### Run ###
        self.rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.loop()
            self.rate.sleep()

    def new_def(self, data):
        self.def_queue.put(data.game_def)
        rospy.loginfo('added game to definition queue: %s', data.game_def)

    def new_command(self, data):
        self.command_queue.put(data.command)
        rospy.loginfo('added command to command queue: %s', data.command)

    def loop(self):
        # rospy.loginfo('running loop with state: %s', self.state)
        # Load in game definition if present. If there are multiple
        # definitions then only take the most recent one.
        new_def = None
        try_again = True
        while try_again:
            try:
                new_def = self.def_queue.get_nowait()
            except queue.Empty:
                try_again = False
        if new_def:
            # if we are loading a new game, we need to get rid of exiting commands
            self.command_queue = queue.Queue()
            self.actions_list = []
            # Eventually we probably want to make this cleaner, but for now I need
            # to get a demo going, so we will manually load in the games
            # TODO: pull games out into database or something
            if new_def == 'simon_says':
                self.actions_list.append(
                    {'speech': 'in simon says, I will tell you something to do and show you how to do it. If I say simon says, you should do it with me. If I do not say simon says, you should not do the action. Watch out, I may try to trick you.'})
                self.set_options(['start'])
                self.set_state(self.states.game_loaded)
                self.action_idx = 0
                rospy.loginfo('ready to play simon says')

        if (self.state == self.states.waiting_for_command
                or self.state == self.states.game_loaded):
            new_command = None
            try:
                new_command = self.command_queue.get_nowait()
            except queue.Empty:
                pass
            if new_command == 'start':
                self.start()
            elif new_command == 'next':
                self.run_next_step()
            elif new_command == 'repeat':
                self.repeat_last_step()
            elif new_command == 'congratulate':
                self.congratulate()
            elif new_command == 'try_again':
                self.try_again()

            # there actions should come in similarly to the way that
            # we see for the movements, where there could be a time
            # delay or no time delay, could be
            # games should have three sections:
            # 1) intro
            # 2) body
            # 3) closing
            # each thing can have a list of options which will can
            # randomly selected.
            # games have some parameters:
            # - randomize body
            #

    def set_state(self, state):
        self.state = state
        self.feedback_pub.publish(state.name)

    def run_step(self, idx):
        this_step = self.actions_list[idx]
        # each step is a dict with:
        # speach, movement or pose
        if 'speech' in this_step:
            speech_goal = SpeechGoal(
                text='<speak>'+this_step['speech']+'</speak>',
                metadata=json.dumps({
                    'text_type': 'ssml',
                    'voice_id': 'Ivy'
                })
            )
            self.speech_server.send_goal(
                speech_goal,
                done_cb=self.speaking_done,
                active_cb=self.speaking_active,
                feedback_cb=self.speaking_feedback)
            self.speaking_state = 'sent'
        if 'move' in this_step:
            move_goal = MoveGoal(this_step['goal'])
            self.move_server.send_goal(
                move_goal,
                done_cb=self.moving_done,
                active_cb=self.moving_active,
                feedback_cb=self.moving_feedback)
            self.moving_state = 'sent'
        if 'pose' in this_step:
            pos_goal = JointTarget()
            pos_goal.name = this_step['pose']['joint_names']
            pos_goal.position = this_step['pose']['joint_positions']
            if this_step['pose']['time']:
                pos_goal.target_completion_time = this_step['pose']['time']
            else:
                pos_goal.target_completion_time = 2
            move_goal = MoveGoal([pos_goal])
            self.move_server.send_goal(
                move_goal,
                done_cb=self.moving_done,
                active_cb=self.moving_active,
                feedback_cb=self.moving_feedback)
            self.moving_state = 'sent'
        self.state = self.states.acting

    def moving_done(self, terminal_state, result):
        self.moving_state = 'done'

    def speaking_done(self, terminal_state, result):
        self.speaking_state = 'done'

    def moving_active(self):
        self.moving_state = 'moving'

    def speaking_active(self):
        self.speaking_state = 'moving'

    def moving_feedback(self, feedback):
        pass

    def speaking_feedback(self, feedback):
        pass

    def set_options(self, options):
        self.command_opts = options
        self.command_opts_pub.publish(options)

    def start(self):
        rospy.loginfo('starting game')
        self.run_step(0)

    def run_next_step(self):
        rospy.loginfo('running next step in game')
        self.action_idx += 1
        self.run_step(self.action_idx)

    def repeat_last_step(self):
        rospy.loginfo('repeating the last step')
        self.run_step(self.action_idx)

    def congratulate(self):
        rospy.loginfo('saying something congratulatory')
        speech_goal = SpeechGoal(random.choice(self.congratulate_strings))
        self.speech_server.send_goal(speech_goal)

    def try_again(self):
        rospy.loginfo('saying to try again and rerunning last step')
        speech_goal = SpeechGoal(random.choice(self.try_again_strings))
        self.speech_server.send_goal(speech_goal)
        self.speech_server.wait_for_result()
        self.repeat_last_step()


if __name__ == '__main__':
    GameRunner()

# there are two components of a game:
#  - the definition, ex: simon says game
#     - some params:
#         - congratualtory speech: list of congratulatory speech uterances
#         - corrective speech: list of corrective things
#     - intro
#         - ex: say_text: in simon says, I will tell you something to do and show you.
#                          if I say "simon says" first, then you should do it too. If
#                          not, then you should stay still. Does that make sense?
#             - options: yes: continue
#                        no:  repeat
#     - body:
#         turn_def:
#             -

#             - feedback:
#                 - next: go to next turn
#                 - next_good_job: say rand_select($congratulatory_speech), go to next turn
#                 - gotcha: say "haha, I Didn't say simon says. Gotcha!!"
#                 - repeat: say "Let's try that again" and go to prior
#     - exit: That was a lot of fun!!

# Note: all of the game should be computed through on load to allow easy traversal

#  - the parameter definition for this particular run:

    # ex_game = {
    #     'parameters': {
    #         'randomize_body': False,
    #         'body_select_ratio': 1,  # what percentage of the body tasks to run
    #     },
    #     'intro': [action sequence ids],
    #     'body': [action sequence ids],
    #     'congratulatory_speech': ['good job', 'that was great', 'you are really good at this'],
    #     'corrective_speech': ['try that again', 'let''s give that another try'],
    #     'closing': [action sequence ids]
    # }

    # simple_simon_says_def = {
    #         'percent_not_simon_says' = 0.2,
    #         'targets': [
    #             {'type': 'action', 'def': id},
    #             {'type': 'pose', 'def': id, 'time': secs},
    #             {'type': 'action', 'def': id}
    #             ],
