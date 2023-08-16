import wx
from utils.storage import LocalStorage

class TabHandEye():
    def __init__(self, parent, tab):
        self.tab = tab
        # db init
        # create a hand eye calib table
        '''
        '''
        self.db = LocalStorage('handeye.db')
        # init ui
        sizer = wx.BoxSizer(wx.VERTICAL)
        self.tab.SetSizer(sizer)
        pass
    