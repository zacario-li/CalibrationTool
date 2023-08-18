import wx
import os
from pathlib import Path
from utils.storage import LocalStorage
from loguru import logger


class TabSingleCam():
    def __init__(self, parent, tab):
        self.tab = tab
        # global var
        self.current_dir = None
        self.db = self.init_db()
        # checkerboard's pattern
        self.checkerboard_row_cell = 0
        self.checkerboard_col_cell = 0
        self.checkerboard_cell_size_in_mm = 0.0

        # init ui
        sizer = wx.BoxSizer(wx.VERTICAL)
        self.tab.SetSizer(sizer)
        self.path_h_sizer = wx.BoxSizer(wx.HORIZONTAL)

        self.m_textCtrl1 = wx.TextCtrl(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_textCtrl1.Enable(False)

        self.path_h_sizer.Add(
            self.m_textCtrl1, 5, wx.ALIGN_CENTER_HORIZONTAL | wx.ALIGN_CENTER_VERTICAL | wx.ALL, 5)

        self.m_select_file_path = wx.Button(
            self.tab, wx.ID_ANY, u"选择文件夹", wx.DefaultPosition, wx.DefaultSize, 0)
        self.path_h_sizer.Add(self.m_select_file_path, 1,
                              wx.ALIGN_CENTER_HORIZONTAL | wx.ALIGN_CENTER_VERTICAL | wx.ALL, 5)

        sizer.Add(self.path_h_sizer, 1, wx.EXPAND, 5)
        self.m_staticline1 = wx.StaticLine(
            self.tab, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL)
        sizer.Add(self.m_staticline1, 0, wx.EXPAND | wx.ALL, 5)
        self.m_listCtrl1 = wx.ListCtrl(
            self.tab, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LC_ICON)
        sizer.Add(self.m_listCtrl1, 14, wx.ALL, 5)

        # register callback
        self.tab.Bind(wx.EVT_BUTTON, self.on_select_file_path,
                      self.m_select_file_path)

    def init_db(self):
        # db init
        # create a single camera calib table
        '''
        use quaternion and position to represent rotation and translation
        |id integer|rootpath text|filename text|isreject bool|qw float |qx float  |qy float  |qz float  |tx float|ty float| tz float|
        |----------|-------------|-------------|-------------|---------|----------|----------|----------|--------|--------|---------|
        |    0     |c:\data\     |    img1.png |  False      |0.1085443|-0.2130855|-0.9618053|-0.1332042| -44.071| 272.898|-1388.602|
        |    1     |c:\data\     |    img2.png |  True       |         |          |          |          |        |        |         |
        '''
        TABLE_SQL_STR = '''id INTEGER PRIMARY KEY AUTOINCREMENT, 
                            rootpath text,
                            filename text, 
                            isreject bool, 
                            qw float, 
                            qx float, 
                            qy float, 
                            qz float, 
                            tx float, 
                            ty float, 
                            tz float'''
        self.DB_FILENAME = 'single.db'
        self.DB_TABLENAME = 'single'
        db = LocalStorage(self.DB_FILENAME)
        ret = db.create_table(self.DB_TABLENAME, TABLE_SQL_STR)
        if ret is not True:
            logger.debug(f'create db table {self.DB_TABLENAME} failed')
            return None
        else:
            return db

    def list_images_with_suffix(self, rootpath: str, suffix_list: list = ['png', 'jpg', 'jpeg', 'bmp']):
        images = []
        for f in os.listdir(rootpath):
            suffix = f.rsplit('.', 1)[-1].lower()
            if suffix in suffix_list:
                images.append(f)
        return images

    def on_select_file_path(self, evt):
        dir_dialog = wx.DirDialog(
            None, "选择校准图像路径", style=wx.DD_DEFAULT_STYLE | wx.DD_NEW_DIR_BUTTON)
        if dir_dialog.ShowModal() == wx.ID_OK:
            self.current_dir = dir_dialog.GetPath()
            self.m_textCtrl1.SetValue(self.current_dir)
        dir_dialog.Destroy()

        images = self.list_images_with_suffix(self.current_dir)
        # progress
        keep_going = True
        count = 0
        max_value = len(images)
        dlg = wx.ProgressDialog("waiting",
                                "图片加载中，请稍后",
                                maximum=max_value,
                                parent=self.tab,
                                style=wx.PD_APP_MODAL)

        for item in images:
            count += 1
            self.db.write_data(
                self.DB_TABLENAME, f'null, \'{self.current_dir}\', \'{item}\', 0, null, null, null, null, null, null, null')
            (keep_going, skip) = dlg.Update(count, f'adding {count} images')
        dlg.Destroy()
