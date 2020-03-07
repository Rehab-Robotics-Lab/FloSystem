#! /usr/bin/env python
""" DataBase Module
This module manages the low level components of the DB for the Lil'Flo system

Classes:
    - DB: The core DB class

"""

from __future__ import print_function
import sqlite3
import os.path
from os import mkdir


class DB(object):
    def __init__(self, db_location):
        db_location = os.path.expanduser(db_location)

        dir_name = os.path.dirname(db_location)
        if not os.path.exists(dir_name):
            mkdir(dir_name)

        try:
            self.conn = sqlite3.connect(db_location)
        except sqlite3.OperationalError as err:
            print("unable to connect to database at location: {}".format(db_location))
            print("error: {}".format(err))
        self.conn.row_factory = sqlite3.Row
        self.__make_db()

    def ex(self, command, *args):
        """execute a command against the databse.
        Abstracts out the execute command, filling in, commiting, and closing the order

        Args:
            command: The command to run
            *args: Any arguments to pass

        Returns: The return from the db
        """
        # this with statement will auto commit
        with self.conn:
            if args:
                to_return = self.conn.execute(command, args)
            else:
                to_return = self.conn.execute(command)
        return to_return

    def drop_table(self, table):
        """drop_table

        Args:
            table: the table to drop
        """
        self.ex('DROP TABLE ?', table)

    def con(self):
        """Return the connection"""
        return self.conn

    def __del__(self):
        self.conn.close()

    def __make_db(self):
        """Make the database structure"""
        self.ex('''CREATE TABLE IF NOT EXISTS poses (
                    id integer PRIMARY KEY,
                    description text,
                    joint_positions text,
                    joint_names text
                );''')

        self.ex('''CREATE TABLE IF NOT EXISTS pose_sequences(
                    id integer PRIMARY KEY,
                    times text, 
                    pose_ids text, 
                    arms text, 
                    total_time real,
                    description text
                );''')

        # type can be either 'ssml' or 'plain'. Removing type
        # always assume ssml. and it is in the metadata anyway
        self.ex('''CREATE TABLE IF NOT EXISTS utterances(
                    id integer PRIMARY KEY,
                    text text, 
                    length real,
                    metadata text
                );''')

        self.ex('''CREATE TABLE IF NOT EXISTS action_sequences (
                    id integer PRIMARY KEY,
                    action_type text,
                    target text,
                    time_to_complete text,
                    wait_for_prior text,
                    description text
                );''')

        # name is the name of the sequence, examples: standard_easy, standard_hard, cp_teen, etc
        # subject is the target subject, for a game that is for any subject, use 0
        # the targeted_game is the game this is for: simon_says or target_touch
        # description is a longer description of the game bucket
        # steps are the actual elements in the bucket, stored as a json array of StepDefs
        self.ex('''CREATE TABLE IF NOT EXISTS game_buckets (
                    id integer PRIMARY KEY,
                    name text,
                    subject integer,
                    targeted_game text,
                    description text,
                    steps text
                );''')
