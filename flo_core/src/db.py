#! /usr/bin/env python

import sqlite3
import pdb
import json


class DB(object):
    def __init__(self, db_location):
        try:
            self.conn = sqlite3.connect(db_location)
        except sqlite3.OperationalError as e:
            print("unable to connect to database at location: {}".format(db_location))
            print("error: {}".format(e))
        self.conn.row_factory = sqlite3.Row

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

    def add_pose(self, description, angles, names):
        self.ex('insert into poses(description, angles, names) values (?,?,?)',
                description, json.dumps(angles), json.dumps(names))
