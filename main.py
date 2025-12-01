import os

os.environ['GTK_THEME'] = 'Adwaita'

import wx
from ui.mainwindow import MainWindow
from loguru import logger


class CalibrationApp(wx.App):
    def OnInit(self):
        self.frame = MainWindow(None, title='CalibrationTool')
        self.SetTopWindow(self.frame)
        self.frame.Show()
        return True


def setup_logging():
    logger.add("app.log", 
               rotation="10 MB",
               format="{time:YYYY-MM-DD HH:mm:ss} | {level} | {message}",
               level="INFO")


if __name__ == '__main__':
    setup_logging()
    app = CalibrationApp()
    app.MainLoop()
