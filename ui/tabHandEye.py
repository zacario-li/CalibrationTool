import wx
from utils.storage import LocalStorage
from loguru import logger


class TabHandEye():
    def __init__(self, parent, tab):
        self.tab = tab
        # global var
        self.db = self.init_db()
        # init ui
        sizer = wx.BoxSizer(wx.VERTICAL)
        self.tab.SetSizer(sizer)
        pass

    def init_db(self):
        # db init
        # create a single camera calib table
        '''
        filename text, isreject bool, qw float, qx float, qy float, qz float, tx float, ty float, tz float
        '''
        TABLE_SQL_STR = 'filename text, isreject bool, qw float, qx float, qy float, qz float, tx float, ty float, tz float'
        self.DB_FILENAME = ':memory:'
        self.DB_TABLENAME = 'handeye'
        db = LocalStorage(self.DB_FILENAME)
        ret = db.create_table(self.DB_TABLENAME, TABLE_SQL_STR)
        if ret is not True:
            logger.debug(f'create db table {self.DB_TABLENAME} failed')
            return None
        else:
            return db
