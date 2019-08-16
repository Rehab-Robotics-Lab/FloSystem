#! /usr/bin/env python

from db import DB


if __name__ == "__main__":
    db_location = '/home/mjsobrep/db/flo.db'
    db = DB(db_location)

    db.ex('''CREATE TABLE poses (
                id integer PRIMARY KEY,
                description text,
                joint_positions text,
                joint_names text
            );''')

    db.ex('''CREATE TABLE pose_sequences(
                id integer PRIMARY KEY,
                times text, 
                pose_ids text, 
                arms text, 
                total_time real,
                description text
            );''')

    db.ex('''CREATE TABLE phrases(
                id integer PRIMARY KEY,
                type text, 
                text text, 
                length real,
                metadata text
            );''')

    db.ex('''CREATE TABLE motion_sequences (
                id integer PRIMARY KEY,
                commands text,
                length real,
                description text
            );''')
