#!/usr/bin/env python

"""A module to generate target touch type games"""

import random
from flo_core_defs.msg import StepDef

DEFAULT_DEF = [
    StepDef(type='pose_left',
            text='Touch the red dot', id=17, time=.7),
    StepDef(type='pose_right',
            text='Touch the green dot', id=17, time=.7),
    StepDef(type='pose_left',
            text='Touch the yellow dot', id=18, time=.7),
    StepDef(type='pose_right',
            text='Touch the blue dot', id=18, time=.7),
]


def target_touch(new_def, process_step, neutral):
    """Generate a target touch game

    Args:
        new_def: The definition of the game
        process_step: The function to process steps into actions

    Returns: The action list that defines the game
    """
    actions_list = []
    actions_list.append(neutral)
    actions_list.append(
        {'speech': 'in the target touch activity, I will tell you to ' +
                   'touch the dots on my hands. We will do 10 ' +
                   'touches, per dot. I will count to tell you when to ' +
                   'go. No tricks here, just good work!! Let\'s start ' +
                   'in a ready position'})
    if not new_def.steps:
        new_def.steps = DEFAULT_DEF

    actions_bag = []
    steps = new_def.steps
    random.shuffle(steps)
    for step in steps:
        targets, speech = process_step(step)

        actions_bag.append(
            {'speech': speech, 'targets': targets})

        actions_bag.extend(
            [{'speech': '{}'.format(idx)} for idx in range(10)])
        actions_bag.append(neutral)

    actions_list += actions_bag

    actions_list.append(
        {'speech': 'that was hard work, but a lot of fun, thanks for playing with me'})
    return actions_list
