import sqlite3
import os


class LocalStorage():
    def __init__(self, database_name: str):
        if os.path.isfile(database_name):
            os.remove(database_name)

        self.db_connection = sqlite3.connect(database_name)
        self.cursor = self.db_connection.cursor()

    def close(self):
        self.db_connection.close()

    def create_table(self, tablename: str, tableformat: str):
        sqlstr = f'CREATE TABLE {tablename} ({tableformat})'

        try:
            self.cursor.execute(sqlstr)
        except sqlite3.OperationalError as e:
            if e.args[0].startswith(f'table {tablename} already exists'):
                return False

        self.db_connection.commit()
        return True

    def retrive_data(self, tablename: str):
        pass

    def write_data(self, tablename: str, datasql: str):
        pass
