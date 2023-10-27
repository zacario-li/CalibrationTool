import wx
import cv2


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

    def set_cvmat(self, mat):
        # TODO bgr2rgb and gray check
        temp_mat = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
        self.bitmap = wx.Bitmap.FromBuffer(temp_mat.shape[1], temp_mat.shape[0], temp_mat)

class DetailsImagePanel(wx.Frame):
    def __init__(self, parent, title, size=(800,600), style=wx.STAY_ON_TOP|wx.FRAME_NO_TASKBAR):
        wx.Frame.__init__(self, parent, title=title, size=size)
        self.SetMinSize(self.GetSize())
        self.SetMaxSize(self.GetSize())

        self.Bind(wx.EVT_ACTIVATE, self._on_activate)
        self.Bind(wx.EVT_CLOSE, self._on_close)

        self._setup_ui()

    def _on_activate(self, evt):
        self.SetFocus()

    def _on_close(self, evt):
        self.GetParent().Enable()
        self.Destroy()

    def _setup_ui(self):
        main_sizer = wx.BoxSizer(wx.VERTICAL)
        self.SetSizer(main_sizer)
        self.Centre(wx.BOTH)
        self.img_panel = ImagePanel(self, self.Size)

    def commit_cvdata(self, cvmat):
        resized_cvmat = cv2.resize(cvmat, self.Size)
        self.img_panel.set_cvmat(resized_cvmat)
    
