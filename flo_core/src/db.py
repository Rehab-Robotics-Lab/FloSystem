#! /usr/bin/env python

from __future__ import print_function
import sqlite3
import pdb
import json
import os.path
from os import mkdir


class DB(object):
    def __init__(self, db_location):
        need_to_build = False
        db_location = os.path.expanduser(db_location)

        dir_name = os.path.dirname(db_location)
        if not os.path.exists(dir_name):
            mkdir(dir_name)

        if not os.path.exists(db_location):
            need_to_build = True

        try:
            self.conn = sqlite3.connect(db_location)
        except sqlite3.OperationalError as e:
            print("unable to connect to database at location: {}".format(db_location))
            print("error: {}".format(e))
        self.conn.row_factory = sqlite3.Row
        if need_to_build:
            self.make_db(db_location)

    def ex(self, command, *args):
        # this with statement will auto commit
        with self.conn:
            if args:
                to_return = self.conn.execute(command, args)
            else:
                to_return = self.conn.execute(command)
        return to_return

    def drop_table(self, table):
        self.ex('DROP TABLE ?', table)

    def con(self):
        return self.conn

    def __del__(self):
        self.conn.close()

    def make_db(self, db_location):
        self.ex('''CREATE TABLE poses (
                    id integer PRIMARY KEY,
                    description text,
                    joint_positions text,
                    joint_names text
                );''')

        self.ex('''CREATE TABLE pose_sequences(
                    id integer PRIMARY KEY,
                    times text, 
                    pose_ids text, 
                    arms text, 
                    total_time real,
                    description text
                );''')

        self.ex('''CREATE TABLE phrases(
                    id integer PRIMARY KEY,
                    type text, 
                    text text, 
                    length real,
                    metadata text
                );''')

        self.ex('''CREATE TABLE action_sequences (
                    id integer PRIMARY KEY,
                    action_type text,
                    target text,
                    time_to_complete text,
                    wait_for_prior text,
                    description text
                );''')
