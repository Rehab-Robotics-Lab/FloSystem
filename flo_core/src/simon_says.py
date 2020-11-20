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


def simon_says(new_def, process_step, neutral):
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
    for step in new_def.steps:
        targets, speech = process_step(step, True)

        actions_bag.append(
            {'speech': 'simon says '+speech, 'targets': targets})
        if random.random() > 0.7:  # this is where we add in non-simon says tasks
            actions_bag.append(
                {'speech': speech, 'targets': targets})

    random.shuffle(actions_bag)
    actions_list += actions_bag

    actions_list.append(
        {'speech': 'that was a lot of fun, thanks for playing with me'})
    actions_list = list(chain.from_iterable(
        (neutral, at) for at in actions_list))
    return actions_list
