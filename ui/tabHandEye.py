import wx
from utils.storage import LocalStorage
from loguru import logger


class TabHandEye():
    def __init__(self, parent, tab):
        self.tab = tab
        # global var
        self.db = self.init_db()
        # init ui
        self.m_layout_he_main = wx.BoxSizer(wx.VERTICAL)
        self.tab.SetSizer(self.m_layout_he_main)
        self.create_main_ui()

    def _create_he_type(self):
        m_layout_he_type = wx.BoxSizer(wx.HORIZONTAL)

        m_radioBoxChoices_calib_type = [u"AX=XB", u"AX=YB"]
        self.m_radioBox_calib_type = wx.RadioBox(self.tab, wx.ID_ANY, u"Calib Type", wx.DefaultPosition,
                                       wx.DefaultSize, m_radioBoxChoices_calib_type, 1, wx.RA_SPECIFY_ROWS)
        self.m_radioBox_calib_type.SetSelection(0)
        m_layout_he_type.Add(self.m_radioBox_calib_type, 0,
                             wx.ALIGN_CENTER_VERTICAL | wx.ALL, 5)

        self.m_checkBox_rotation_only = wx.CheckBox(
            self.tab, wx.ID_ANY, u"R Only", wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_he_type.Add(self.m_checkBox_rotation_only, 0,
                             wx.ALIGN_CENTER_VERTICAL | wx.ALL, 0)
        return m_layout_he_type

    def _create_he_calib_method(self):
        m_layout_he_calib_method = wx.BoxSizer(wx.HORIZONTAL)

        m_radioBoxChoices_axxb_calib_method = [u"TSAI", u"PARK",
                              u"HORAUD", u"ANDREFF", u"DANIILIDIS"]
        self.m_radioBox_axxb_calib_method = wx.RadioBox(self.tab, wx.ID_ANY, u"AX=XB Calib Method",
                                       wx.DefaultPosition, wx.DefaultSize, m_radioBoxChoices_axxb_calib_method, 1, wx.RA_SPECIFY_ROWS)
        self.m_radioBox_axxb_calib_method.SetSelection(2)
        m_layout_he_calib_method.Add(self.m_radioBox_axxb_calib_method, 0, wx.ALL, 5)

        m_radioBoxChoices_axyb_calib_method = [u"SHAH", u"LI"]
        self.m_radioBox_axyb_calib_method = wx.RadioBox(self.tab, wx.ID_ANY, u"AX=YB Calib Method",
                                       wx.DefaultPosition, wx.DefaultSize, m_radioBoxChoices_axyb_calib_method, 1, wx.RA_SPECIFY_ROWS)
        self.m_radioBox_axyb_calib_method.SetSelection(0)
        self.m_radioBox_axyb_calib_method.Enable(False)

        m_layout_he_calib_method.Add(self.m_radioBox_axyb_calib_method, 0, wx.ALL, 5)
        return m_layout_he_calib_method

    def _create_he_dataloader(self):
        m_layout_he_dataloader = wx.BoxSizer(wx.HORIZONTAL)

        m_layout_he_dataloader_path_main = wx.BoxSizer(wx.VERTICAL)

        m_layout_he_dataloader_A = wx.BoxSizer(wx.HORIZONTAL)

        self.m_textCtrl_load_a_path = wx.TextCtrl(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_he_dataloader_A.Add(self.m_textCtrl_load_a_path, 10, wx.ALL, 1)
        self.m_textCtrl_load_a_path.Enable(False)

        self.m_btn_loadA = wx.Button(
            self.tab, wx.ID_ANY, u"Load A", wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_he_dataloader_A.Add(self.m_btn_loadA, 0, wx.ALL, 1)

        m_layout_he_dataloader_path_main.Add(
            m_layout_he_dataloader_A, 10, wx.EXPAND, 0)

        m_layout_he_dataloader_B = wx.BoxSizer(wx.HORIZONTAL)

        self.m_textCtrl_load_b_path = wx.TextCtrl(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_he_dataloader_B.Add(self.m_textCtrl_load_b_path, 10, wx.ALL, 1)
        self.m_textCtrl_load_b_path.Enable(False)

        self.m_btn_loadB = wx.Button(
            self.tab, wx.ID_ANY, u"Load B", wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_he_dataloader_B.Add(self.m_btn_loadB, 0, wx.ALL, 1)

        m_layout_he_dataloader_path_main.Add(
            m_layout_he_dataloader_B, 10, wx.EXPAND, 0)

        m_layout_he_dataloader_param = wx.BoxSizer(wx.HORIZONTAL)

        self.m_textCtrl_cam_param_path = wx.TextCtrl(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_he_dataloader_param.Add(self.m_textCtrl_cam_param_path, 10, wx.ALL, 1)
        self.m_textCtrl_cam_param_path.Enable(False)

        self.m_btn_load_cam_param = wx.Button(
            self.tab, wx.ID_ANY, u"Load Camera Params", wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_he_dataloader_param.Add(self.m_btn_load_cam_param, 0, wx.ALL, 1)

        m_layout_he_dataloader_path_main.Add(
            m_layout_he_dataloader_param, 10, wx.EXPAND, 0)

        m_layout_he_dataloader.Add(
            m_layout_he_dataloader_path_main, 10, wx.EXPAND, 0)

        self.m_btn_calib = wx.Button(
            self.tab, wx.ID_ANY, u"Calib", wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_he_dataloader.Add(self.m_btn_calib, 0, wx.ALL | wx.EXPAND, 1)

        self.m_btn_save = wx.Button(
            self.tab, wx.ID_ANY, u"Save", wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_he_dataloader.Add(self.m_btn_save, 1, wx.ALL | wx.EXPAND, 1)
        return m_layout_he_dataloader

    def _create_he_view(self):
        m_layout_he_view = wx.BoxSizer(wx.VERTICAL)

        self.m_bitmap_checkview = wx.StaticBitmap(
            self.tab, wx.ID_ANY, wx.NullBitmap, wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_he_view.Add(self.m_bitmap_checkview, 0, wx.ALL, 5)

        self.m_statictext_calib_err_result = wx.StaticText(
            self.tab, wx.ID_ANY, u"MyLabel", wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_statictext_calib_err_result.Wrap(-1)

        m_layout_he_view.Add(self.m_statictext_calib_err_result, 0, wx.ALL, 5)
        return m_layout_he_view

    def create_main_ui(self):
        # 手眼标定类型选择区域
        m_layout_he_type = self._create_he_type()
        self.m_layout_he_main.Add(m_layout_he_type, 1, wx.EXPAND, 0)

        # 手眼标定方法选择区域
        m_layout_he_calib_method = self._create_he_calib_method()
        self.m_layout_he_main.Add(m_layout_he_calib_method, 2, wx.EXPAND, 0)

        # 手眼标定数据选择区域
        self.m_staticline_dataloader_upper = wx.StaticLine( self.tab, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
        self.m_layout_he_main.Add( self.m_staticline_dataloader_upper, 0, wx.EXPAND |wx.ALL, 0 )
        
        m_layout_he_dataloader = self._create_he_dataloader()
        self.m_layout_he_main.Add(m_layout_he_dataloader, 2, wx.EXPAND, 0)

        self.m_staticline_dataloader_lower = wx.StaticLine( self.tab, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
        self.m_layout_he_main.Add( self.m_staticline_dataloader_lower, 0, wx.EXPAND |wx.ALL, 0 )

        # 手眼标定结果查看区域
        m_layout_he_view = self._create_he_view()        
        self.m_layout_he_main.Add(m_layout_he_view, 20, wx.EXPAND, 0)

    def init_db(self):
        # db init
        # create a single camera calib table
        '''
        rootpath text ,filename text, isreject bool, qw float, qx float, qy float, qz float, tx float, ty float, tz float
        '''
        TABLE_SQL_STR = 'filename text, isreject bool, qw float, qx float, qy float, qz float, tx float, ty float, tz float'
        self.DB_FILENAME = ':memory:'
        self.DB_TABLENAME = 'handeye'
        db = LocalStorage(self.DB_FILENAME)
        ret = db.create_table(self.DB_TABLENAME, TABLE_SQL_STR)
        if ret is not True:
            logger.debug(f'create db table {self.DB_TABLENAME} failed')
            return None
        else:
            return db
