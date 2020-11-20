#!/usr/bin/env python
"""Game Runner for the Flo Robot

This module manages the state machine that runs the
games on the flo robot. It is all accessed by ROS.

Publishers:
    - /game_runner_state : This is feedback on the
                              state of the game and how
                              it is progressing. Publishes
                              messages of type GameState
    - /game_runner_command_opts: These are the options
                                 which can be passed to
                                 the game runner to control
                                 progress through the game.
                                 publishes messges of type
                                 GameCommandOptions
    - /game_runner_actions: This reports what the game runner
                            is sending out to have acted on,
                            both individual speech and full
                            steps with both speech and joint
                            targets

Subscribers:
    - /game_runner_commands: Takes the commands to control the
                             game. Accepts messages of type
                             GameCommand
    - /game_runner_def: Takes the definition for the game. For right
                 now this is just a string naming the game.
                 Will be more robust later. Takes messages
                 of type GameDef

Neccesary Action Servers:
    - tts: the speech server from tts-ros
    - move: the movement server from the flo_humanoid

"""

from enum import Enum
import json
import random
try:
    import queue
except ImportError:
    import Queue as queue
import threading
import os.path
import os
import datetime
import rospy
import actionlib
from tts.msg import SpeechAction, SpeechGoal
from flo_humanoid_defs.msg import MoveAction, MoveGoal, JointTarget
from flo_core_defs.msg import GameState, GameCommandOptions, GameDef,\
    GameCommand, StepDef
from flo_core_defs.srv import GetPoseID
from flo_core_defs.srv import GetPoseSeqID
from flo_core_defs.msg import GameAction
from simon_says import simon_says
from target_touch import target_touch


class GameRunner(object):
    """The actual class that orchestrates the games"""

    # pylint: disable=too-many-instance-attributes
    # having 12 instance attributes is just the way it is for something of this
    # size

    # pylint: disable=too-few-public-methods
    # This class communicates with the ros system, it is not meant to be
    # used directly in client code

    states = Enum(
        'states', 'waiting_for_def game_loaded acting waiting_for_command')

    action_states = Enum('action_states', 'sent active done none')

    congratulate_strings = ['good job', 'that was great', 'well done',
                            'you are doing really well',
                            'you are super good at this']

    try_again_strings = ['let\'s try that again together',
                         'see if you can do a bit better',
                         'how about we give that another try']

    def __init__(self):
        rospy.init_node('game_runner')

        # -- Action Servers to Make Things Happen -- #
        # set up polly action server
        self.speech_server = actionlib.SimpleActionClient(
            'tts', SpeechAction)
        self.speech_server.wait_for_server()

        # setup movement action server
        self.move_server = actionlib.SimpleActionClient('move', MoveAction)
        self.move_server.wait_for_server()

        ### Publishers ###
        self.feedback_pub = rospy.Publisher('game_runner_state',
                                            GameState, queue_size=1, latch=True)
        self.command_opts_pub = rospy.Publisher('game_runner_command_opts',
                                                GameCommandOptions, queue_size=1, latch=True)
        self.game_action_pub = rospy.Publisher(
            'game_runner_actions', GameAction, queue_size=10)
        rospy.loginfo('setup publishers')

        # -- Subscribers -- #
        rospy.Subscriber(
            'game_runner_commands', GameCommand, self.__new_command)
        rospy.Subscriber(
            'game_runner_def', GameDef, self.__new_def)
        rospy.Subscriber(
            'game_runner_def_save', GameDef, self.__generate_game_printout)
        rospy.loginfo('setup subscribers')

        # -- Services -- #
        rospy.wait_for_service('get_pose_id')
        self.get_pose_id = rospy.ServiceProxy('get_pose_id', GetPoseID)
        rospy.wait_for_service('get_pose_seq_id')
        self.get_pose_seq_id = rospy.ServiceProxy(
            'get_pose_seq_id', GetPoseSeqID)
        rospy.loginfo('setup services')

        # -- Queue's for safe receiving of info -- #
        self.command_queue = queue.Queue()
        self.command_lock = threading.Lock()
        self.def_queue = queue.Queue()

        # -- State Management -- #
        self.__set_state(self.states.waiting_for_def)
        self.moving_state = self.action_states.none
        self.speaking_state = self.action_states.none
        self.state = self.states.waiting_for_command

        # -- The actions that compose this game -- #
        self.actions_list = []
        self.action_idx = -1
        self.command_opts = []

        # -- Run -- #
        rate = rospy.Rate(20)
        rospy.loginfo('done with initialization')
        while not rospy.is_shutdown():
            self.__loop()
            rate.sleep()

    def __new_def(self, msg):
        """Add a newly received game def to the game def queue.

        Args:
            msg: The rosmsgs containing the new definition
        """
        self.def_queue.put(msg)
        rospy.loginfo('added game to definition queue: %s', msg.game_type)

    def __new_command(self, msg):
        """Add a newly received command to the command queue

        Args:
            msg: The rosmsg containing the new command
        """
        with self.command_lock:
            if self.command_queue.empty() and msg.command in self.command_opts:
                self.command_queue.put(msg.command)
                rospy.loginfo(
                    'added command to command queue: %s', msg.command)
            else:
                rospy.logerr('got a new command but already have commands')

    def __loop(self):
        """Loop through reading all of the queues and taking the
        necessary actions"""
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
            self.__process_def(new_def)

        if (self.state == self.states.waiting_for_command
                or self.state == self.states.game_loaded):
            new_command = None
            try:
                with self.command_lock:
                    new_command = self.command_queue.get_nowait()
            except queue.Empty:
                pass
            if new_command in self.command_opts:
                self.__process_command(new_command)

        if (self.state == self.states.acting
                and (self.moving_state == self.action_states.done
                     or self.moving_state == self.action_states.none)
                and (self.speaking_state == self.action_states.done
                     or self.speaking_state == self.action_states.none)):
            if self.action_idx+1 == len(self.actions_list):
                self.__set_state(self.states.waiting_for_command)
                self.__set_options(
                    ['repeat', 'congratulate', 'try_again', 'finish_game'])
            else:
                self.state = self.states.waiting_for_command
                self.__set_options(
                    ['next', 'repeat', 'congratulate', 'try_again',
                     'quit_game'])

    def __process_step(self, step, mirror_arms=False):
        targets = []
        speech = step.text
        step_time = step.time if step.time and step.time > 0 else 2
        if step.type == 'pose_left':
            pose = self.get_pose_id(step.id).pose  # type: Pose
            arm = 'right' if mirror_arms else 'left'
            targets = [self.__construct_joint_target(
                pose.joint_names, pose.joint_positions, step_time, arm)]
            # speech = speech+' with your left hand'
        elif step.type == 'pose_right':
            pose = self.get_pose_id(step.id).pose  # type: Pose
            arm = 'left' if mirror_arms else 'right'
            targets = [self.__construct_joint_target(
                pose.joint_names, pose.joint_positions, step_time, arm)]
            # speech = speech+' with your right hand'
        elif step.type == 'pose_both':
            pose = self.get_pose_id(step.id).pose  # type: Pose
            targets = [
                self.__construct_joint_target(
                    pose.joint_names,
                    pose.joint_positions,
                    step_time,
                    'left'),
                self.__construct_joint_target(
                    pose.joint_names,
                    pose.joint_positions,
                    step_time,
                    'right'
                )
            ]
            # speech = speech+' with your right hand'
        elif step.type == 'move':
            sequence = self.get_pose_seq_id(
                step.id).sequence  # type: PoseSeq
            time = 0
            for idx in range(len(sequence.pose_ids)):
                pose = self.get_pose_id(sequence.pose_ids[idx]).pose
                time += sequence.times[idx]
                def_arm = sequence.arms[idx]
                if def_arm == 'left':
                    arm = 'right' if mirror_arms else 'left'
                elif def_arm == 'right':
                    arm = 'left' if mirror_arms else 'right'
                else:
                    rospy.logerr(
                        'an invalid arm was passed through a sequence in game runner')

                target = self.__construct_joint_target(
                    pose.joint_names, pose.joint_positions,
                    time, arm)
                targets.append(target)
        return targets, speech

    def __generate_game_printout(self, new_def):
        targ, spch = self.__process_step(
            StepDef(type='pose_both', id=1, time=1))
        neutral = {'speech': spch, 'targets': targ}
        # Eventually we probably want to make this cleaner, but for now I need
        # to get a demo going, so we will manually load in the games
        if new_def.game_type == 'simon_says':
            actions_list = simon_says(
                new_def, self.__process_step, neutral)
        elif new_def.game_type == 'target_touch':
            actions_list = target_touch(
                new_def, self.__process_step, neutral)
        path = os.path.expanduser('~/flo_games/')
        if not os.path.exists(path):
            os.makedirs(path)
        path = os.path.join(path,
                            '{}-{}'.format(
                                new_def.game_type,
                                datetime.datetime.now().strftime("%Y%m%d-%H%M%S")))
        rospy.loginfo('generating new game printout at %s', path)
        with open(path, 'w') as fhd:
            for action in actions_list:
                if 'speech' in action.keys():
                    fhd.write(action['speech'])
                    fhd.write('\n')

    def __process_def(self, new_def):
        """Process a new game definition

        Args:
            new_def: the game definition
        """
        # if we are loading a new game, we need to get rid of exiting commands
        with self.command_lock:
            self.command_queue = queue.Queue()
        self.actions_list = []
        targ, spch = self.__process_step(
            StepDef(type='pose_both', id=1, time=1))
        neutral = {'speech': spch, 'targets': targ}
        # Eventually we probably want to make this cleaner, but for now I need
        # to get a demo going, so we will manually load in the games
        if new_def.game_type == 'simon_says':
            self.actions_list = simon_says(
                new_def, self.__process_step, neutral)
        elif new_def.game_type == 'target_touch':
            self.actions_list = target_touch(
                new_def, self.__process_step, neutral)

        self.__set_options(['start'])
        self.__set_state(self.states.game_loaded)
        self.action_idx = 0
        rospy.loginfo('ready to play game')

    @staticmethod
    def __construct_joint_target(names, joint_positions, time, arm):
        target = JointTarget()
        target.name = [arm+'_'+nm for nm in names]
        target.position = joint_positions
        target.target_completion_time = time
        return target

    def __process_command(self, new_command):
        """Process a new command

        Args:
            new_command: the new command
        """
        if new_command == 'start':
            self.__start()
        elif new_command == 'next':
            self.__run_next_step()
        elif new_command == 'repeat':
            self.__repeat_last_step()
        elif new_command == 'congratulate':
            self.__congratulate()
        elif new_command == 'try_again':
            self.__try_again()
        elif new_command == 'quit_game':
            self.__quit_game()
        elif new_command == 'finish_game':
            self.__finish_game()

    def __quit_game(self):
        """Quit the game, which essentially just means to set the
        state to be waiting for a definition"""
        self.__set_state(self.states.waiting_for_def)
        self.__set_options([])

    def __finish_game(self):
        """Finish the game, which essentially just means to set the
        state to be waiting for a definition"""
        self.__set_state(self.states.waiting_for_def)
        self.__set_options([])

    def __set_state(self, state):
        """Sets the state of the game. This stores both an internal
        state and publishes the state out as feedback.

        Args:
            state: The state to set, should be a member of the
                   states enum.
        """
        self.state = state
        self.feedback_pub.publish(state.name)

    def __say_plain_text(self, to_say):
        speech_goal = SpeechGoal(
            text='<speak>'+to_say+'</speak>',
            metadata=json.dumps({
                'text_type': 'ssml',
                'voice_id': 'Salli'  # 'Justin' #'Ivy'
            })
        )
        self.speech_server.send_goal(
            speech_goal,
            done_cb=self.__speaking_done,
            active_cb=self.__speaking_active,
            feedback_cb=self.__speaking_feedback
        )
        self.speaking_state = self.action_states.sent
        action = GameAction()
        action.speech = to_say
        self.game_action_pub.publish(action)

    def __run_step(self, idx):
        """Runs a specified step, sending out the necessary actions
        and setting up the callbacks.

        Args:
            idx: the id of the step to run.
        """
        this_step = self.actions_list[idx]
        command_sent = False
        action = GameAction()
        action.step_id = idx
        # each step is a dict with:
        # speach, movement or pose
        if 'speech' in this_step and this_step['speech'] != '':
            self.__say_plain_text(this_step['speech'])
            command_sent = True
            action.speech = this_step['speech']
        if 'targets' in this_step:
            move_goal = MoveGoal(this_step['targets'])
            self.move_server.send_goal(
                move_goal,
                done_cb=self.__moving_done,
                active_cb=self.__moving_active,
                feedback_cb=self.__moving_feedback
            )
            self.moving_state = self.action_states.sent
            command_sent = True
            action.targets = this_step['targets']
        if not command_sent:
            rospy.logerr(
                'tried to run a step that did not have any useful info')
        else:
            self.game_action_pub.publish(action)
            self.__set_state(self.states.acting)
            self.__set_options([])

    def __moving_done(self, *_):
        self.moving_state = self.action_states.done

    def __speaking_done(self, *_):
        self.speaking_state = self.action_states.done

    def __moving_active(self):
        self.moving_state = self.action_states.active

    def __speaking_active(self):
        self.speaking_state = self.action_states.active

    def __moving_feedback(self, feedback):
        pass

    def __speaking_feedback(self, feedback):
        pass

    def __set_options(self, options):
        self.command_opts = options
        self.command_opts_pub.publish(options)

    def __start(self):
        rospy.loginfo('starting game')
        self.__run_step(0)

    def __run_next_step(self):
        rospy.loginfo('running next step in game')
        self.action_idx += 1
        self.__run_step(self.action_idx)

    def __repeat_last_step(self):
        rospy.loginfo('repeating the last step')
        self.action_idx -= 1
        self.__run_step(self.action_idx)
        with self.command_lock:
            self.command_queue.put('next')

    def __congratulate(self):
        rospy.loginfo('saying something congratulatory')
        self.__say_plain_text(random.choice(self.congratulate_strings))
        self.__set_state(self.states.acting)
        self.__set_options([])

    def __try_again(self):
        rospy.loginfo('saying to try again and rerunning last step')
        self.__say_plain_text(random.choice(self.try_again_strings))
        self.speech_server.wait_for_result()
        self.__repeat_last_step()


if __name__ == '__main__':
    GameRunner()
