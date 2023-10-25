import wx
from ui.mainwindow import MainWindow


class CalibrationApp(wx.App):
    def OnInit(self):
        self.frame = MainWindow(None, title='CalibrationTool')
        self.SetTopWindow(self.frame)
        self.frame.Show()
        return True


if __name__ == '__main__':
    app = CalibrationApp()
    app.MainLoop()
