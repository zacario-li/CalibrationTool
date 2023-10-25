import wx

class CustomEvent(wx.PyCommandEvent):
    def __init__(self, evt_type):
        wx.PyCommandEvent.__init__(self, evt_type)
        self.data = None 

    def set_data(self, data):
        self.data = data

    def get_data(self):
        return self.data