#!/usr/bin/env python

"""Module to generate printouts of games"""

import rospy
from flo_core_defs.srv import SearchGameBucket
from flo_core_defs.msg import GameDef

# pylint: disable=superfluous-parens


def run():
    """Get user input and request generation of a game printout"""
    rospy.init_node('game_generator')

    rospy.loginfo('node up, waiting for services')

    rospy.wait_for_service('search_game_bucket_name_desc')
    search_gb = rospy.ServiceProxy(
        'search_game_bucket_name_desc', SearchGameBucket)

    resp = search_gb('')
    print('**** Available Game Buckets: ***')
    for idx, bucket in enumerate(resp.game_buckets):
        # TODO: allow filtering by type or subj
        print('{}: {}'.format(idx, bucket.name))

    good = False
    while not good:
        target = raw_input(
            '\nType in number of game bucket to generate from and press ' +
            'enter. Enter -1 to cancel:\n')
        try:
            bucket_id = int(target)
        except ValueError:
            print('\ninvalid input')
            continue
        if bucket_id == -1:
            return
        if bucket_id >= 0 and bucket_id < len(resp.game_buckets):
            good = True

    good = False
    while not good:
        target = raw_input(
            '\nEnter:\n\ts: simon says\n\tt: target touch\n\tq: quit\n')

        if target == 's':
            game_type = 'simon_says'
            good = True
        elif target == 't':
            game_type = 'target_touch'
            good = True
        elif target == 'q':
            return

    print('\ngenerating for a {} game with bucket: {}'.format(
        game_type, bucket_id))

    game_def = GameDef(game_type=game_type,
                       steps=resp.game_buckets[bucket_id].steps,
                       reps=3
                       min_steps=2
                       max_steps=4
                       bimanual=true)

    pub = rospy.Publisher('game_runner_def_save',
                          GameDef, queue_size=0, latch=True)
    pub.publish(game_def)

    print('done')
    rospy.spin()


if __name__ == '__main__':
    run()
