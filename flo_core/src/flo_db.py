#! /usr/bin/env python

import sqlite3
import pdb
import json

# Needs to be able to:
# - search for pose by id
# - search for pose by string
# - gep pose seq by id
# - search for pose seq by string
# - add pose <- if ID specified, replace
# - add pose seq <- if ID specified, replace


class FLO_DB(object):
    def __init__(
        rospy.init_node('db_manager')

        db_path=rospy.get_param("database_location")
        self.db=DB(db_path)

        self.
