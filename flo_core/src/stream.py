from flo_core_defs.msg import StepDef


def stream(process_step, neutral):
    actions_list = []
    # actions_list.append(neutral)
    actions_list.append(
        {'speech': 'Hi I\'m Flo.  For this activity we are going to be doing some actions. If I do an action, please do it with me. I will demonstrate all movements mirrored. Are you ready to get started?'})
    targ, _ = process_step(
        StepDef(type='pose_both', id=1, time=1))
    actions_list.append(
        {'speech': 'first raise your arms above your head', 'targets': targ})
    targ, _ = process_step(
        StepDef(type='pose_both', id=7, time=1))
    actions_list.append(
        {'speech': 'Extend your arms as far as they can go toward the ceiling', 'targets': targ})
    targ, _ = process_step(
        StepDef(type='pose_both', id=3, time=1))
    actions_list.append(
        {'speech': 'shrug your shoulders as high as you can', 'targets': targ})
    targ, _ = process_step(
        StepDef(type='pose_both', id=13, time=1))
    actions_list.append(
        {'speech': 'Now touch the top of your head with your right hand'})
    targ, _ = process_step(
        StepDef(type='pose_both', id=1, time=1))
    return actions_list
