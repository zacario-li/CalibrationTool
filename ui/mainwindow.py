import wx
from .tabSingleCam import TabSingleCam
from .tabStereoCam import TabStereoCam
from .tabHandEye import TabHandEye


class MainWindow(wx.Frame):
    def __init__(self, parent, title):
        wx.Frame.__init__(self, parent, title=title, size=(1280, 900))
        self.SetMinSize(self.GetSize())
        self.SetMaxSize(self.GetSize())
        self.Centre(wx.BOTH)
        # icon
        icon = wx.Icon('elements/calib_icon.png', wx.BITMAP_TYPE_PNG)
        self.SetIcon(icon)

        # Create notebook with 3 tabs
        self.notebook = wx.Notebook(self)

        tabsingle = wx.Panel(self.notebook)
        tabstereo = wx.Panel(self.notebook)
        tabhandeye = wx.Panel(self.notebook)

        self.notebook.AddPage(tabsingle, 'Mono Camera')
        self.notebook.AddPage(tabstereo, 'Stereo Camera')
        self.notebook.AddPage(tabhandeye, 'HandEye')

        # init tabs
        self.tsinglecam = TabSingleCam(self, tabsingle)
        self.tstereocam = TabStereoCam(self, tabstereo)
        self.thandeye = TabHandEye(self, tabhandeye)
