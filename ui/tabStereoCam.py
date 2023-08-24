import wx
from utils.storage import LocalStorage
from ui.imagepanel import ImagePanel
from loguru import logger


class TabStereoCam():
    def __init__(self, parent, tab):
        self.tab = tab
        # global var
        self.db = self.init_db()
        self.current_leftroot = ''
        self.current_rightroot = ''
        self.current_row_cors = ''
        self.current_col_cors = ''
        self.current_cellsize = ''
        # init ui
        sizer = wx.BoxSizer(wx.VERTICAL)
        self.tab.SetSizer(sizer)
        # path load layout:h
        self.m_layout_path_load = self._create_path_load_layout()
        sizer.Add(self.m_layout_path_load, 1, wx.EXPAND, 5)

        # action btns layout:h
        self.m_layout_actions_btns = self._create_actions_btns_layout()
        sizer.Add(self.m_layout_actions_btns, 1, wx.EXPAND, 5)

        # seprate line
        self.m_staticline_seprate = wx.StaticLine(
            self.tab, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL)
        sizer.Add(self.m_staticline_seprate, 0, wx.EXPAND | wx.ALL, 5)

        # main view layout:v
        self.m_layout_main_view = self._create_main_view_layout(
            wx.Size(480, 270))
        sizer.Add(self.m_layout_main_view, 20, wx.EXPAND, 5)

        # callbacks
        self._register_all_callbacks()

    def _create_path_load_layout(self):
        m_layout_path_load = wx.BoxSizer(wx.HORIZONTAL)
        # . sub path load layout:v
        self.m_layout_sub_path_load = wx.BoxSizer(wx.VERTICAL)
        m_layout_path_load.Add(self.m_layout_sub_path_load, 8, border=5)

        self.m_textctl_left_path = wx.TextCtrl(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_textctl_left_path.Enable(False)
        self.m_textctl_right_path = wx.TextCtrl(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_textctl_right_path.Enable(False)

        self.m_layout_sub_path_load.Add(self.m_textctl_left_path, 0, wx.EXPAND)
        self.m_layout_sub_path_load.Add(
            self.m_textctl_right_path, 0, wx.EXPAND)

        self.m_btn_load_files = wx.Button(
            self.tab, wx.ID_ANY, u"选择文件夹&&设置参数", wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_path_load.Add(self.m_btn_load_files, 1, wx.EXPAND, border=5)
        return m_layout_path_load

    def _create_actions_btns_layout(self):
        m_layout_actions_btns = wx.BoxSizer(wx.HORIZONTAL)
        self.m_statictext_warning = wx.StaticText(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 | wx.BORDER_SIMPLE)
        self.m_statictext_warning.Wrap(-1)

        m_layout_actions_btns.Add(
            self.m_statictext_warning, 12, wx.EXPAND, 5)

        self.m_btn_calibrate = wx.Button(
            self.tab, wx.ID_ANY, u"开始标定", wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_actions_btns.Add(
            self.m_btn_calibrate, 1, wx.EXPAND, 5)

        self.m_btn_save_calibration = wx.Button(
            self.tab, wx.ID_ANY, u"保存标定结果", wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_actions_btns.Add(
            self.m_btn_save_calibration, 1, wx.EXPAND, 5)
        return m_layout_actions_btns

    def _create_main_view_layout(self, bitmapsize: wx.Size):
        m_layout_main_view = wx.BoxSizer(wx.HORIZONTAL)
        # .
        self.m_layout_tree = wx.BoxSizer(wx.HORIZONTAL)
        self.m_layout_stereo_image_view = wx.BoxSizer(wx.HORIZONTAL)
        # ..
        self.m_layout_left_image = wx.BoxSizer(wx.VERTICAL)
        self.m_layout_right_image = wx.BoxSizer(wx.VERTICAL)

        # . layout
        self.m_treectrl = self.create_treectrl()
        self.m_layout_tree.Add(self.m_treectrl, 1, wx.EXPAND, 5)

        # . layout
        self.m_statictext_left_name = wx.StaticText(
            self.tab, wx.ID_ANY, u"left name", wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_bitmap_left = ImagePanel(self.tab, bitmapsize)
        self.m_layout_left_image.Add(
            self.m_statictext_left_name, 0, wx.ALIGN_CENTER | wx.ALL, 5)
        self.m_layout_left_image.Add(
            self.m_bitmap_left, 5, wx.ALIGN_CENTER_HORIZONTAL, 5)

        self.m_statictext_right_name = wx.StaticText(
            self.tab, wx.ID_ANY, u"right name", wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_bitmap_right = ImagePanel(self.tab, bitmapsize)
        self.m_layout_right_image.Add(
            self.m_statictext_right_name, 0, wx.ALIGN_CENTER | wx.ALL, 5)
        self.m_layout_right_image.Add(
            self.m_bitmap_right, 5, wx.ALIGN_CENTER_HORIZONTAL, 5)

        self.m_layout_stereo_image_view.Add(
            self.m_layout_left_image, 1, wx.EXPAND, 5)
        self.m_layout_stereo_image_view.Add(
            self.m_layout_right_image, 1, wx.EXPAND, 5)

        # .. sub image view layout:v X 2
        m_layout_main_view.Add(self.m_layout_tree, 1, wx.EXPAND, 5)
        m_layout_main_view.Add(
            self.m_layout_stereo_image_view, 4, wx.EXPAND, 5)
        return m_layout_main_view

    def _init_checkerboard_loader(self, parent, pp):
        dlg_file_loader = StereoFileLoader(parent, pp)
        return dlg_file_loader

    def _register_all_callbacks(self):
        self.tab.Bind(wx.EVT_BUTTON, self.on_open_file_loader,
                      self.m_btn_load_files)

    def init_db(self):
        # db init
        # create a stereo camera calib table
        '''
        use quaternion and position to represent rotation and translation
        |id integer|rootpath text|cameraid int|filename text|isreject bool|qw float |qx float  |qy float  |qz float  |tx float|ty float| tz float|  rpje|
        |----------|-------------|------------|-------------|-------------|---------|----------|----------|----------|--------|--------|---------|------|
        |    0     |c:\data\L    |0           |    img1.png |  False      |0.1085443|-0.2130855|-0.9618053|-0.1332042| -44.071| 272.898|-1388.602|0.1826|
        |    1     |c:\data\R    |1           |    img1.png |  True       |         |          |          |          |        |        |         |      |
        '''
        TABLE_SQL_STR = '''id INTEGER PRIMARY KEY AUTOINCREMENT, 
                            rootpath text,
                            cameraid int,
                            filename text, 
                            isreject bool, 
                            qw float, 
                            qx float, 
                            qy float, 
                            qz float, 
                            tx float, 
                            ty float, 
                            tz float,
                            rpje float'''
        self.DB_FILENAME = 'stereo.db'
        self.DB_TABLENAME = 'stereo'
        db = LocalStorage(self.DB_FILENAME)
        ret = db.create_table(self.DB_TABLENAME, TABLE_SQL_STR)
        if ret is not True:
            logger.debug(f'create db table {self.DB_TABLENAME} failed')
            return None
        else:
            return db

    # 左侧树形目录显示每张标定结果的控件
    def create_treectrl(self):
        tree = wx.TreeCtrl(self.tab)
        # tree.AssignImageList(self.iconlist)
        return tree

    # 更新目录条目信息 TODO
    def update_treectrl(self, all: bool = False):
        tree = self.m_treeCtl_images
        tree.DeleteAllItems()
        # retrive qualified images from db
        if all is False:
            condi = f'WHERE isreject=0'
        else:
            condi = ''
        results = self.db.retrive_data(
            self.DB_TABLENAME, f'rootpath, filename, rpje', condi)
        filelist = [f[1] for f in results]
        rpjes = [r[2] for r in results]
        dirroot = tree.AddRoot('文件名:(重投影误差)', image=0)
        if len(filelist) > 0:
            for fname, r in zip(filelist, rpjes):
                newItem = tree.AppendItem(dirroot, f'{fname}:({str(r)})')
                tree.SelectItem(newItem)
                tree.EnsureVisible(newItem)
                tree.EnableVisibleFocus(True)
                tree.SetItemImage(newItem, self.icon_ok)
            tree.Expand(dirroot)

    def on_open_file_loader(self, evt):
        dlg_file_loader = self._init_checkerboard_loader(None, self)
        dlg_file_loader.ShowModal()


class StereoFileLoader(wx.Dialog):
    def __init__(self, parent, pp):
        wx.Dialog.__init__(self, parent, title='标定图像数据设置',
                           size=wx.Size(650, 200))
        self.SetMinSize(self.GetSize())
        self.SetMaxSize(self.GetSize())
        self.pp = pp

        # temp l/r path
        self.temp_lpath = ''
        self.temp_rpath = ''

        # init ui
        self.m_layout_dlg_main = wx.BoxSizer(wx.VERTICAL)
        self.SetSizer(self.m_layout_dlg_main)

        self.m_statictext_left_label = wx.StaticText(
            self, wx.ID_ANY, u" left", wx.DefaultPosition, wx.DefaultSize, 5)
        self.m_layout_dlg_main.Add(
            self.m_statictext_left_label, 0, wx.EXPAND, 5)

        self.m_layout_left_file = wx.BoxSizer(wx.HORIZONTAL)

        self.m_textctl_left_path = wx.TextCtrl(
            self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_textctl_left_path.Enable(False)
        self.m_btn_left = wx.Button(
            self, wx.ID_ANY, u'选择左相机图片', wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_layout_left_file.Add(self.m_textctl_left_path, 5, wx.EXPAND, 5)
        self.m_layout_left_file.Add(self.m_btn_left, 0, wx.EXPAND, 5)

        self.m_layout_dlg_main.Add(self.m_layout_left_file, 0, wx.EXPAND, 5)

        # .
        self.m_statictext_right_label = wx.StaticText(
            self, wx.ID_ANY, u" right", wx.DefaultPosition, wx.DefaultSize, 5)
        self.m_layout_right_file = wx.BoxSizer(wx.HORIZONTAL)
        self.m_layout_dlg_main.Add(self.m_statictext_right_label)

        self.m_textctl_right_path = wx.TextCtrl(
            self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_textctl_right_path.Enable(False)
        self.m_btn_right = wx.Button(
            self, wx.ID_ANY, u'选择右相机图片', wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_layout_right_file.Add(
            self.m_textctl_right_path, 5, wx.EXPAND, 5)
        self.m_layout_right_file.Add(self.m_btn_right, 0, wx.EXPAND, 5)
        self.m_layout_dlg_main.Add(self.m_layout_right_file, 0, wx.EXPAND, 5)

        self.m_staticline = wx.StaticLine(
            self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL)
        self.m_layout_dlg_main.Add(self.m_staticline, 0, wx.EXPAND, 5)

        # parameter
        self.m_layout_pattern_parameter = wx.BoxSizer(wx.HORIZONTAL)
        self.m_layout_dlg_main.Add(self.m_layout_pattern_parameter)

        self.m_staticText_row = wx.StaticText(
            self, wx.ID_ANY, u"标定板行数", wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_staticText_row.Wrap(-1)

        self.m_layout_pattern_parameter.Add(
            self.m_staticText_row, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 5)

        self.m_textCtrl_row = wx.TextCtrl(
            self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_layout_pattern_parameter.Add(
            self.m_textCtrl_row, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 5)

        self.m_staticText_col = wx.StaticText(
            self, wx.ID_ANY, u"标定板列数", wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_staticText_col.Wrap(-1)

        self.m_layout_pattern_parameter.Add(
            self.m_staticText_col, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 5)

        self.m_textCtrl_col = wx.TextCtrl(
            self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_layout_pattern_parameter.Add(
            self.m_textCtrl_col, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 5)

        self.m_staticText_cellsize = wx.StaticText(
            self, wx.ID_ANY, u"标定板单元格边长(mm)", wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_staticText_cellsize.Wrap(-1)

        self.m_layout_pattern_parameter.Add(
            self.m_staticText_cellsize, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 5)

        self.m_textCtrl_cellsize = wx.TextCtrl(
            self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_layout_pattern_parameter.Add(
            self.m_textCtrl_cellsize, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 5)

        m_layout_dlgbtnsizer = wx.StdDialogButtonSizer()
        self.m_btn_sdbOK = wx.Button(self, wx.ID_OK)
        m_layout_dlgbtnsizer.AddButton(self.m_btn_sdbOK)
        self.m_btn_sdbCancel = wx.Button(self, wx.ID_CANCEL)
        m_layout_dlgbtnsizer.AddButton(self.m_btn_sdbCancel)
        m_layout_dlgbtnsizer.Realize()
        self.m_layout_dlg_main.Add(m_layout_dlgbtnsizer, 1, wx.EXPAND, 5)
        self.m_staticline2 = wx.StaticLine(
            self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL)
        self.m_layout_dlg_main.Add(self.m_staticline2, 0, wx.EXPAND, 5)

        # init parameter
        self._setup_parameter()
        # register callbacks
        self._register_callbacks()

    def on_select_file_path(self, evt):
        source_btn_id = evt.GetId()

        dir_dialog = wx.DirDialog(
            None, "选择校准图像路径", style=wx.DD_DEFAULT_STYLE | wx.DD_NEW_DIR_BUTTON)
        if dir_dialog.ShowModal() == wx.ID_OK:
            if source_btn_id == self.m_btn_left.GetId():
                self.temp_lpath = dir_dialog.GetPath()
                self.m_textctl_left_path.SetValue(self.temp_lpath)
            elif source_btn_id == self.m_btn_right.GetId():
                self.temp_rpath = dir_dialog.GetPath()
                self.m_textctl_right_path.SetValue(self.temp_rpath)
        else:
            return
        dir_dialog.Destroy()

    def on_confirm_parameter(self, evt):
        lp = self.m_textctl_left_path.GetValue()
        rp = self.m_textctl_right_path.GetValue()
        col = self.m_textCtrl_col.GetValue()
        row = self.m_textCtrl_row.GetValue()
        cell = self.m_textCtrl_cellsize.GetValue()
        self.pp.current_leftroot = lp
        self.pp.current_rightroot = rp
        self.pp.current_row_cors = row
        self.pp.current_col_cors = col
        self.pp.current_cellsize = cell
        self.Destroy()

    def on_text_enter(self, evt):
        lp = self.m_textctl_left_path.GetValue()
        rp = self.m_textctl_right_path.GetValue()
        col = self.m_textCtrl_col.GetValue()
        row = self.m_textCtrl_row.GetValue()
        cell = self.m_textCtrl_cellsize.GetValue()

        if len(lp) > 0 and len(rp) > 0 and len(col) > 0 and len(row) > 0 and len(cell) > 0:
            self.m_btn_sdbOK.Enable()
        else:
            self.m_btn_sdbOK.Enable(False)

    def _register_callbacks(self):
        self.Bind(wx.EVT_BUTTON, self.on_select_file_path, self.m_btn_left)
        self.Bind(wx.EVT_BUTTON, self.on_select_file_path, self.m_btn_right)
        self.Bind(wx.EVT_BUTTON, self.on_confirm_parameter, self.m_btn_sdbOK)
        self.Bind(wx.EVT_TEXT, self.on_text_enter, self.m_textCtrl_row)
        self.Bind(wx.EVT_TEXT, self.on_text_enter, self.m_textCtrl_col)
        self.Bind(wx.EVT_TEXT, self.on_text_enter, self.m_textCtrl_cellsize)

    def _setup_parameter(self):
        self.m_textctl_left_path.SetValue(self.pp.current_leftroot)
        self.m_textctl_right_path.SetValue(self.pp.current_rightroot)
        self.m_textCtrl_col.SetValue(self.pp.current_row_cors)
        self.m_textCtrl_row.SetValue(self.pp.current_col_cors)
        self.m_textCtrl_cellsize.SetValue(self.pp.current_cellsize)

        # check if all parameters are set
        lp = self.m_textctl_left_path.GetValue()
        rp = self.m_textctl_right_path.GetValue()
        col = self.m_textCtrl_col.GetValue()
        row = self.m_textCtrl_row.GetValue()
        cell = self.m_textCtrl_cellsize.GetValue()
        if len(lp) > 0 and len(rp) > 0 and len(col) > 0 and len(row) > 0 and len(cell) > 0:
            self.m_btn_sdbOK.Enable()
        else:
            self.m_btn_sdbOK.Enable(False)
