import wx
from .tab1 import Tab1

class MainWindow(wx.Frame):
    def __init__(self, parent, title):
        wx.Frame.__init__(self, parent, title=title, size=(800, 600))
        
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
        
        tab1 = wx.Panel(self.notebook)
        tab2 = wx.Panel(self.notebook)
        tab3 = wx.Panel(self.notebook)
        
        self.notebook.AddPage(tab1, '单目相机')
        self.notebook.AddPage(tab2, '双目相机') 
        self.notebook.AddPage(tab3, '手眼标定')
        
        # Add horizontal box sizer to each tab
        tab1_sizer = wx.BoxSizer(wx.HORIZONTAL)
        tab1.SetSizer(tab1_sizer)
        
        tab2_sizer = wx.BoxSizer(wx.HORIZONTAL)
        tab2.SetSizer(tab2_sizer)
        
        tab3_sizer = wx.BoxSizer(wx.HORIZONTAL)
        tab3.SetSizer(tab3_sizer)

        # prepare tab1
        self.t1 = Tab1(self, tab1, tab1_sizer)
 