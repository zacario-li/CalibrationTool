import wx


class ImagePanel(wx.Panel):
    def __init__(self, parent, size=wx.DefaultSize):
        wx.Panel.__init__(self, parent, size=size)
        self.bitmap = wx.Bitmap(size)
        self.Bind(wx.EVT_PAINT, self.on_paint)

    def on_paint(self, evt):
        dc = wx.PaintDC(self)
        dc.Clear()
        dc.DrawBitmap(self.bitmap, 0, 0)

    def set_bitmap(self, bitmap):
        self.bitmap = bitmap
