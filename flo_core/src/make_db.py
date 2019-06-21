#!/usr/bin/env python

from db import DB


if __name__ == "__main__":
    db_location = '/home/mjsobrep/db/flo.db'
    db = DB(db_location)

    db.ex('''CREATE TABLE poses (
                id integer PRIMARY KEY,
                description text,
                poses text,
                names text
            );''')

    db.ex('''CREATE TABLE sequences (
                id integer PRIMARY KEY,
                times real,
                commands text,
                length real
            );''')

    db.ex('''CREATE TABLE phrases(
                id integer PRIMARY KEY,
                type text, 
                text text, 
                length real,
                metadata text
            );''')
