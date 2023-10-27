import wx

class CustomEvent(wx.PyCommandEvent):
    def __init__(self, evt_type):
        wx.PyCommandEvent.__init__(self, evt_type)
        self.data = None 

    def set_data(self, data):
        self.data = data

    def get_data(self):
        return self.data
    
class ImagePanel(wx.Panel):
    def __init__(self, parent, size):
        wx.Panel.__init__(self, parent, size=size)
        self.bitmap = wx.Bitmap(size)
        self.Bind(wx.EVT_PAINT, self.on_paint)

    def on_paint(self, evt):
        dc = wx.PaintDC(self)
        dc.Clear()
        dc.DrawBitmap(self.bitmap, 0, 0)

    def set_bitmap(self, bitmap):
        self.bitmap = bitmap