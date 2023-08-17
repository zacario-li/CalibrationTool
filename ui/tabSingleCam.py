import wx
from utils.storage import LocalStorage
from loguru import logger


class TabSingleCam():
    def __init__(self, parent, tab):
        self.tab = tab
        # global var
        self.current_dir = None
        self.db = self.init_db()
        ## checkerboard's pattern
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
        filename text, isreject bool, qw float, qx float, qy float, qz float, tx float, ty float, tz float
        '''
        TABLE_SQL_STR = 'filename text, isreject bool, qw float, qx float, qy float, qz float, tx float, ty float, tz float'
        self.DB_FILENAME = 'single.db'
        self.DB_TABLENAME = 'single'
        db = LocalStorage(self.DB_FILENAME)
        ret = db.create_table(self.DB_TABLENAME, TABLE_SQL_STR)
        if ret is not True:
            logger.debug(f'create db table {self.DB_TABLENAME} failed')
            return None
        else:
            return db

    def on_select_file_path(self, evt):
        dir_dialog = wx.DirDialog(
            None, "选择校准图像路径", style=wx.DD_DEFAULT_STYLE | wx.DD_NEW_DIR_BUTTON)
        if dir_dialog.ShowModal() == wx.ID_OK:
            self.current_dir = dir_dialog.GetPath()
            self.m_textCtrl1.SetValue(self.current_dir)
        dir_dialog.Destroy()
        # progress
        keep_going = True
        count = 0
        max_value = 100
        dlg = wx.ProgressDialog("waiting",
                                "图片加载中，请稍后",
                                maximum=max_value,
                                parent=self.tab,
                                style=wx.PD_CAN_ABORT | wx.PD_APP_MODAL | wx.PD_ELAPSED_TIME | wx.PD_REMAINING_TIME)
        while keep_going and count < max_value:
            count += 1
            wx.Sleep(1)
            (keep_going, skip) = dlg.Update(count)
        dlg.Destroy()
