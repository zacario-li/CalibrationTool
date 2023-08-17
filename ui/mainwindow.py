import wx
from .tabSingleCam import TabSingleCam
from .tabStereoCam import TabStereoCam
from .tabHandEye import TabHandEye


class MainWindow(wx.Frame):
    def __init__(self, parent, title):
        wx.Frame.__init__(self, parent, title=title, size=(1280, 900))
        self.SetMinSize(self.GetSize())
        self.SetMaxSize(self.GetSize())

        # Create menu bar
        self.menu_bar = wx.MenuBar()

        file_menu = wx.Menu()
        new_item = file_menu.Append(wx.ID_NEW, 'New')
        open_item = file_menu.Append(wx.ID_OPEN, 'Open')
        save_item = file_menu.Append(wx.ID_SAVE, 'Save')
        exit_item = file_menu.Append(wx.ID_EXIT, 'Exit')

        edit_menu = wx.Menu()
        cut_item = edit_menu.Append(wx.ID_CUT, 'Cut')
        copy_item = edit_menu.Append(wx.ID_COPY, 'Copy')
        paste_item = edit_menu.Append(wx.ID_PASTE, 'Paste')

        help_menu = wx.Menu()
        about_item = help_menu.Append(wx.ID_ABOUT, 'About')

        self.menu_bar.Append(file_menu, 'File')
        self.menu_bar.Append(edit_menu, 'Edit')
        self.menu_bar.Append(help_menu, 'Help')
        self.SetMenuBar(self.menu_bar)

        # Create notebook with 3 tabs
        self.notebook = wx.Notebook(self)

        tabsingle = wx.Panel(self.notebook)
        tabstereo = wx.Panel(self.notebook)
        tabhandeye = wx.Panel(self.notebook)

        self.notebook.AddPage(tabsingle, '单目相机')
        self.notebook.AddPage(tabstereo, '双目相机')
        self.notebook.AddPage(tabhandeye, '手眼标定')

        # init tabs
        self.tsinglecam = TabSingleCam(self, tabsingle)
        self.tstereocam = TabStereoCam(self, tabstereo)
        self.thandeye = TabHandEye(self, tabhandeye)
