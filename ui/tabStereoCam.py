import wx
from utils.storage import LocalStorage

class TabStereoCam():
    def __init__(self, parent, tab):
        self.tab = tab
        # db init
        # create a stereo camera calib table
        '''
        '''
        self.db = LocalStorage('stereo.db')
        # init ui
        sizer = wx.BoxSizer(wx.VERTICAL)
        self.tab.SetSizer(sizer)
        pass