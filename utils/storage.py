import sqlite3
import os

class LocalStorage():
    def __init__(self, database_name:str):
        if os.path.isfile(database_name):
            os.remove(database_name)
            
        self.db_connection = sqlite3.connect(database_name)
        self.cursor = self.db_connection.cursor()
        
        # create a single camera calib table
        '''
        filename text, isreject bool, qw float, qx float, qy float, qz float, tx float, ty float, tz float
        '''
        self.cursor.execute('''CREATE TABLE single
                            (filename text, isreject bool, qw float, qx float, qy float, qz float, tx float, ty float, tz float)''')
        self.db_connection.commit()

        # create a stereo calib table

        # create a hand eye calib table
    
    def close(self):
        self.db_connection.close()
