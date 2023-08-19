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

    # result = retrive_data(self.DB_TABLENAME, '*')
    def retrive_data(self, tablename: str, datasql: str, condsql:str=''):
        full_sql = f'SELECT {datasql} FROM {tablename} {condsql}'
        self.cursor.execute(full_sql)
        records = self.cursor.fetchall()
        return records

    def write_data(self, tablename: str, datasql: str):
        full_sql = f'''INSERT INTO {tablename} 
                            VALUES ({datasql})'''
        self.cursor.execute(full_sql)
        self.db_connection.commit()

    # modify_data(self.DB_TABLENAME, f'SET qw=0.123456, qx= 1.987654 WHERE filename=\'20230607_154004.png\'')
    def modify_data(self, tablename: str, datasql: str):
        full_sql = f'UPDATE {tablename} ' + datasql
        self.cursor.execute(full_sql)
        self.db_connection.commit()

    # delete_data(self.DB_TABLENAME, f'WHERE filename=\'20230607_154448.png\'')
    def delete_data(self, tablename: str, datasql: str):
        full_sql = f'DELETE FROM {tablename} ' + datasql
        self.cursor.execute(full_sql)
        self.db_connection.commit()
