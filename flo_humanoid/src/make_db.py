import sqlite3
import pdb


class DB(object):
    def __init__(self, db_location):
        self.conn = sqlite3.connect(db_location)

    def ex(self, command):
        # In theory this with statement will auto commit
        with self.conn:
            curs = self.conn.cursor()
            curs.execute(command)

    def __del__(self):
        self.conn.close()


if __name__ == "__main__":
    db_location = '/home/mjsobrep/db/test.db'
    db = DB(db_location)

    db.ex('''CREATE TABLE test (
                id integer PRIMARY KEY,
                description text
            );''')
