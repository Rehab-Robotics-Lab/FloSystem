#!/usr/bin/env python

import sqlite3
import pdb


class DB(object):
    def __init__(self, db_location):
        self.conn = sqlite3.connect(db_location)
        self.conn.row_factory = sqlite3.Row

    def ex(self, command, *args):
        # this with statement will auto commit
        with self.conn:
            self.conn.execute(command, args)

    def drop_table(self, table):
        self.ex('DROP TABLE ?', table)

    def __del__(self):
        self.conn.close()
