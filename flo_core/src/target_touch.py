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


def target_touch(new_def, process_step):
    """Generate a target touch game

    Args:
        new_def: The definition of the game
        process_step: The function to process steps into actions

    Returns: The action list that defines the game
    """
    actions_list = []
    reps = 10
    if new_def.reps:
        reps = new_def.reps

    actions_list.append(
        {'speech': 'in the target touch activity, I will tell you to ' +
                   'touch one of the dots on my hands. Each time I extend ' +
                   'the hand, you should touch it. We will do {} touches '.format(reps) +
                   'per dot. No tricks here, just good work!! Let\'s start ' +
                   'in a ready position'})
    if not new_def.steps:
        new_def.steps = DEFAULT_DEF

    actions_bag = []
    for step in new_def.steps:
        targets, speech = process_step(step)

        actions_bag.extend(
            [{'speech': speech, 'targets': targets} for x in range(10)])

    random.shuffle(actions_bag)
    actions_list += actions_bag

    actions_list.append(
        {'speech': 'that was hard work, but a lot of fun, thanks for playing with me'})
    return actions_list
