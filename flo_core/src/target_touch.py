#!/usr/bin/env python

"""A module to generate target touch type games"""

import random


def target_touch(new_def, process_step, neutral):
    """Generate a target touch game

    Args:
        new_def: The definition of the game
        process_step: The function to process steps into actions

    Returns: The action list that defines the game
    """
    actions_list = []
    sides = {'blue': ('left', 'upper'), 'red': ('left', 'lower'), 'yellow': (
        'right', 'upper'), 'green': ('right', 'lower')}

    actions_list.append(neutral)
    actions_list.append(
        {'speech': 'in the target touch activity, I will tell you to ' +
                   'touch the dots on my hands. I will tell you which ' +
                   'hand to use and which color dot to touch, then tell you to go. No tricks ' +
                   'here, just good work!! Let\'s start ' +
                   'in a ready position, return to this position after every touch'})

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
                obj_to_add = {'targets': targets,
                              'speech': speech, 'color': color}
                moves[left_right][up_down] = moves[left_right][up_down] + \
                    new_def.reps*[obj_to_add]

    random.shuffle(moves['left']['upper'])
    random.shuffle(moves['left']['lower'])
    random.shuffle(moves['right']['upper'])
    random.shuffle(moves['right']['lower'])
    while (moves['left']['upper'] or
           moves['left']['lower'] or
           moves['right']['upper'] or
           moves['right']['lower']):
        # how many instructions to give
        num_steps = random.randrange(new_def.min_steps, new_def.max_steps+1)
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

        left_remain = left_ud is not None
        right_remain = right_ud is not None
        seq = []
        left_target = None
        right_target = None
        for _ in range(num_steps):
            if left_remain and (random.getrandbits(1) or not right_remain):
                try:
                    new_move = moves['left'][left_ud].pop()
                    left_target = new_move['targets']
                    seq.append(new_move)
                except (IndexError, KeyError):
                    left_remain = False
            elif right_remain:
                try:
                    new_move = moves['right'][right_ud].pop()
                    right_target = new_move['targets']
                    seq.append(new_move)
                except (IndexError, KeyError):
                    right_remain = False

        if left_target and right_target:
            final_target = left_target+right_target
        elif left_target:
            final_target = left_target
        elif right_target:
            final_target = right_target
        else:
            raise Exception

        final_speech = '<break time="2.5s"/>'
        for item in seq:
            if 'left' in item['speech']:
                final_speech = final_speech + \
                    '<emphasis>left</emphasis> hand <emphasis>' + \
                    item['color'] + '</emphasis>, '
            elif 'right' in item['speech']:
                final_speech = final_speech + \
                    '<emphasis>right</emphasis> hand <emphasis>' + \
                    item['color'] + '</emphasis>, '
            else:
                raise Exception

        actions_bag.append(
            {'speech': final_speech+'<break time=".5s"/> go!', 'targets': final_target})

    random.shuffle(actions_bag)
    actions_list += actions_bag

    actions_list.append(
        {'speech': 'that was hard work, but a lot of fun, thanks for playing with me'})
    # actions_list = list(chain.from_iterable(
    #     (neutral, at) for at in actions_list))
    return actions_list
