#!/usr/bin/env python

"""A module to generate Stream type games"""


from flo_core_defs.msg import StepDef


def stream(process_step, neutral):
    """STREAM test. provides the stream test for the flo robot

    Args:
        process_step: Function to generate targets and text from a step definition
        neutral: The neutral action definition for the robot
    """
    actions_list = []
    actions_list.append(neutral)
    actions_list.append(
        {'speech': 'Hi I\'m Flo. For this activity we are going to be doing some actions. ' +
                   'If I do an action, please do it with me. I will demonstrate all the ' +
                   'actions before you try them. Are you ready to get started?'})
    targ, _ = process_step(
        StepDef(type='pose_both', id=5, time=1))
    actions_list.append(
        {'speech': 'first raise your arms above your head', 'targets': targ})
    targ, _ = process_step(
        StepDef(type='pose_both', id=5, time=1))
    actions_list.append(
        {'speech': 'Extend your arms as far as you can toward the ceiling', 'targets' : targ})
    targ, _ = process_step(
        StepDef(type='pose_both', id=1, time=1))
    actions_list.append(
        {'speech': 'shrug your shoulders as high as you can', 'targets': targ})
    targ, _ = process_step(
        StepDef(type='pose_both', id=9, time=1))
    actions_list.append(
        {'speech': 'Now touch the top of your head with both hands', 'targets' : targ})
    targ, _ = process_step(
        StepDef(type='pose_both', id=7, time=1))
    actions_list.append(
        {'speech': 'Finally reach forward towards me with both arms', 'target' : targ}) 
    targ, _ = process_step(
        StepDef(type='pose_both', id=1, time=1)),
    actions_list.append(
        {'speech': 'Thanks for playing with me', 'target' : targ})
    targ, _ = process_step(
        StepDef(type='pose_both', id=22, time=1))
    return actions_list
