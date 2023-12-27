from utils.storage import LocalStorage
import numpy as np
import pickle

def test():
    TABLE_SQL_STR = '''id INTEGER PRIMARY KEY AUTOINCREMENT, 
                            rootpath text,
                            filename text, 
                            isreject bool, 
                            qw float, 
                            qx float, 
                            qy float, 
                            qz float, 
                            tx float, 
                            ty float, 
                            tz float,
                            rpje float,
                            cors blob'''
    #self.DB_FILENAME = ':memory:'
    DB_FILENAME = 'single.db'
    DB_TABLENAME = 'single'
    db = LocalStorage(DB_FILENAME)
    ret = db.create_table(DB_TABLENAME, TABLE_SQL_STR)
    db.write_data(
                DB_TABLENAME, f'null, null, null, 0, null, null, null, null, null, null, null, null, null')
    new_array = np.random.rand(88,2).astype('float32')
    array_bytes = pickle.dumps(new_array)
    db.modify_data(DB_TABLENAME, f'''SET cors=?, filename=? WHERE isreject=0 ''', (array_bytes, DB_TABLENAME))
    results = db.retrive_data(DB_TABLENAME, 'filename, cors', 'WHERE isreject=0')
    cors = [c[1] for c in results]
    r_a = pickle.loads(cors[0])
    print(np.array_equal(new_array, r_a))
    pass

if __name__ == '__main__':
    test()