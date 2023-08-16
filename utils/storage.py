import sqlite3
import os

class LocalStorage():
    def __init__(self, database_name:str, table_string:str=None):
        if os.path.isfile(database_name):
            os.remove(database_name)

        self.db_connection = sqlite3.connect(database_name)
        self.cursor = self.db_connection.cursor()
        
        if table_string is not None:
            self.cursor.execute(table_string)
            self.db_connection.commit()
    
    def close(self):
        self.db_connection.close()
