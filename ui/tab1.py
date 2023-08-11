import wx
import wx.dataview as dv

class Tab1():
    def __init__(self, parent, tab, sizer):

        # Create tree ctrl, bitmap, and vtk window
        parent.tree = dv.DataViewTreeCtrl(tab)
        parent.bitmap = wx.StaticBitmap(tab) 
        
        # Set up proportions for horizontal sizer
        sizer.Add(parent.tree, proportion=1, flag=wx.EXPAND)
        sizer.Add(wx.StaticLine(tab), flag=wx.EXPAND|wx.LEFT|wx.RIGHT, border=5) 
        sizer.Add(parent.bitmap, proportion=2, flag=wx.EXPAND)