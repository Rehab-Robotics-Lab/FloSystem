#!/usr/bin/env python
"""A module for generating simon says type games"""

import random
from itertools import chain


# ok to have too many args for a utility func
# pylint: disable=too-many-arguments
def sort_defs(new_def, left, right, process_step, check_side_seq, append_action):
    """Sort the definitions from a new game definition into the correct
    arms/sequences with speech

    Args:
        new_def: the game definition
        left: sequences on the left arm (add to this)
        right: sequences on the right arm (add to this)
        process_step: Function to process the step to get out the targets and speech
        check_side_seq: Function to find out which side a sequence of motions operates on
        append_action: Add the action to the action bag, handling any specialization for game
    """
    for step in new_def.steps:
        targets, speech = process_step(step, True)
        if step.type == 'pose_right':
            right.append({'targets': targets, 'speech': speech})
        elif step.type == 'pose_left':
            left.append({'targets': targets, 'speech': speech})
        elif step.type == 'move':
            side = check_side_seq(step.id)
            if side == 'right':
                right.append({'targets': targets, 'speech': speech})
            elif side == 'left':
                left.append({'targets': targets, 'speech': speech})
            elif side == 'both':
                append_action(targets, speech)
            else:
                raise Exception
        else:
            raise Exception


def mix_bimanual(left, right, append_action):
    """Mix up actions to make them bimanual

    Args:
        left: The left arm actions
        right: The right arm actions
        append_action: function to append actions to the final game sequence
    """
    while left and right:
        left_act = left.pop()
        right_act = right.pop()
        if random.getrandbits(1):
            speech = left_act['speech'] + ' and ' + right_act['speech']
        else:
            speech = right_act['speech'] + ' and ' + left_act['speech']
        targets = left_act['targets']+right_act['targets']
        targets.sort(key=lambda target: target.target_completion_time)
        append_action(targets, speech)


def simon_says(new_def, process_step, check_side_seq, neutral):
    """Generate a simon says game

    Args:
        new_def: The game bucket definition to use
        process_step: The function which can be used to process steps into actions

    Returns: The action list that defines the game
    """
    actions_list = []
    actions_list.append(
        {'speech': 'in simon says, I will tell you something to do and ' +
                   'show you how to do it, mirrored. If I say simon says, you ' +
                   'should do it with me. If I do not say simon says, you should ' +
                   'not do the action. Watch out, I may try to trick you. ' +
                   'After every movement return to a ready position'})
    actions_bag = []
    left = []
    right = []

    def append_action(targets, speech):
        """Append a new action to the bag of actions, adding a non-simon
        says action with 30% frequency

        Args:
            targets: Target to move arms to
            speech: Speech to speak
        """
        actions_bag.append(
            {'speech': 'simon says '+speech, 'targets': targets})
        if random.random() > 0.7:  # this is where we add in non-simon says tasks
            actions_bag.append(
                {'speech': speech, 'targets': targets})

    sort_defs(new_def, left, right, process_step,
              check_side_seq,  append_action)
    random.shuffle(left)
    random.shuffle(right)
    if new_def.bimanual:
        mix_bimanual(left, right, append_action)
    # If either we didn't run in bimanual mode or if there is just some left over in one arm:
    while left:
        left_act = left.pop()
        append_action(left_act['targets'], left_act['speech'])
    while right:
        right_act = right.pop()
        append_action(right_act['targets'], right_act['speech'])

    random.shuffle(actions_bag)
    actions_list += actions_bag

    actions_list.append(
        {'speech': 'that was a lot of fun, thanks for playing with me'})
    actions_list = list(chain.from_iterable(
        (neutral, at) for at in actions_list))
    return actions_list
