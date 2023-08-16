import wx

class TabStereoCam():
    def __init__(self, parent, tab):
        self.tab = tab
        # init ui
        sizer = wx.BoxSizer(wx.VERTICAL)
        self.tab.SetSizer(sizer)
        pass