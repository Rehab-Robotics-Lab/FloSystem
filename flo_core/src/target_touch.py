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
    reps = 10
    sides = {'blue': ('left', 'top'), 'red': ('left', 'bottom'), 'yellow': (
        'right', 'top'), 'green': ('right', 'bottom')}
    if new_def.reps:
        reps = new_def.reps

    actions_list.append(neutral)
    actions_list.append(
        {'speech': 'in the target touch activity, I will tell you to ' +
                   'touch the dots on my hands. I will tell you which ' +
                   'hand to use and which color dot to touch then tell you to go. No tricks ' +
                   'here, just good work!! Let\'s start ' +
                   'in a ready position, return to this position after every touch'})
    if not new_def.steps:
        new_def.steps = DEFAULT_DEF

    actions_bag = []
    steps = new_def.steps
    random.shuffle(steps)
    moves = {
        'left': {
            'upper': [],
            'lower': []
        },
        'right': {
            'upper': [],
            'lower': []

        }}
    for step in steps:
        targets, speech = process_step(step)
        for color in sides:
            if color in speech:
                color_tuple = sides[color]
                left_right = color_tuple[0]
                up_down = color_tuple[1]
                for _ in range(reps):
                    moves[left_right][up_down] = {'targets': targets,
                                                  'speech': speech, 'color': color}
    while (moves['left']['upper'] or
            moves['left']['upper'] or
            moves['left']['upper'] or
            moves['left']['upper']):
        num_steps = random.randrange(2, 5)
        left_ud = None
        right_ud = None

        if moves['left']['upper'] and (random.getrandbits(1) or not moves['left']['lower']):
            left_ud = 'upper'
        elif moves['left']['lower']:
            left_ud = 'lower'

        if moves['right']['upper'] and (random.getrandbits(1) or not moves['right']['lower']):
            right_ud = 'upper'
        elif moves['right']['lower']:
            right_ud = 'lower'

        left_remain = True
        right_remain = True
        seq = []
        left_target = None
        right_target = None
        for _ in range(num_steps):
            if left_remain and random.getrandbits(1):
                try:
                    new_move = moves['left'][left_ud]
                    left_target = new_move['targets']
                    seq.append(new_move)
                except IndexError:
                    left_remain = False
            elif right_remain:
                try:
                    new_move = moves['right'][right_ud]
                    right_target = new_move['targets']
                    seq.append(new_move)
                except IndexError:
                    right_remain = False

        if left_target and right_target:
            final_target = left_target+right_target
        elif left_target:
            final_target = left_target
        elif right_target:
            final_target = right_target
        else:
            raise Exception

        final_speech = ''
        for item in seq:
            if 'left' in item['speech']:
                final_speech = final_speech + \
                    'left hand ' + item['color'] + ' '
            elif 'right' in item['speech']:
                final_speech = final_speech + \
                    'right hand ' + item['color'] + ' '
            else:
                raise Exception

        actions_bag.append(
            {'speech': final_speech+' go!', 'targets': final_target})

    actions_list += actions_bag

    actions_list.append(
        {'speech': 'that was hard work, but a lot of fun, thanks for playing with me'})
    return actions_list
