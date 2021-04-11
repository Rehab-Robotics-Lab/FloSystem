#!/usr/bin/env python
"""A module for generating simon says type games"""

import random
from itertools import chain
from flo_core_defs.msg import StepDef

DEFAULT_DEF = [
    StepDef(type='move', text='wave', id=2),
    StepDef(type='move', text='clap your hands', id=6),
    StepDef(type='move', text='disco', id=8),
    StepDef(
        type='pose_left', text='raise your left arm straight out in front', id=2),
    StepDef(
        type='pose_right', text='raise your right arm straight out in front', id=2),
    StepDef(
        type='pose_right',
        text='raise your right arm straight out to the side', id=3),
    StepDef(
        type='pose_left',
        text='raise your left arm straight out to the side', id=3),
    StepDef(
        type='pose_left',
        text='touch the top of your head with your left hand', id=11),
    StepDef(
        type='pose_right',
        text='touch the top of your head with your right hand', id=11),
    StepDef(type='pose_both', text='cover your eyes!', id=16),
]


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
    if not new_def.steps:
        new_def.steps = DEFAULT_DEF
    actions_bag = []
    left = []
    right = []

    def append_action(targets, speech):
        actions_bag.append(
            {'speech': 'simon says '+speech, 'targets': targets})
        if random.random() > 0.7:  # this is where we add in non-simon says tasks
            actions_bag.append(
                {'speech': speech, 'targets': targets})

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
    random.shuffle(left)
    random.shuffle(right)
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
