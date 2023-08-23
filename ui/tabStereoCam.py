import wx
from utils.storage import LocalStorage
from ui.imagepanel import ImagePanel
from loguru import logger


class TabStereoCam():
    def __init__(self, parent, tab):
        self.tab = tab
        # global var
        self.db = self.init_db()
        # init ui
        sizer = wx.BoxSizer(wx.VERTICAL)
        self.tab.SetSizer(sizer)
        # path load layout:h
        self.m_layout_path_load = self._create_path_load_layout()
        sizer.Add(self.m_layout_path_load, 1, wx.EXPAND, 5)

        # action btns layout:h
        self.m_layout_actions_btns = self._create_actions_btns_layout()
        sizer.Add(self.m_layout_actions_btns, 1, wx.EXPAND, 5)

        # main view layout:v
        self.m_layout_main_view = self._create_main_view_layout(wx.Size(480,270))
        sizer.Add(self.m_layout_main_view, 20, wx.EXPAND, 5)

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
            self.tab, wx.ID_ANY, u"选择文件夹", wx.DefaultPosition, wx.DefaultSize, 0)
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

    def _create_main_view_layout(self, bitmapsize:wx.Size):
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
        self.m_layout_left_image.Add(self.m_bitmap_left, 5, wx.ALIGN_CENTER_HORIZONTAL, 5)

        self.m_statictext_right_name = wx.StaticText(
            self.tab, wx.ID_ANY, u"right name", wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_bitmap_right = ImagePanel(self.tab, bitmapsize)
        self.m_layout_right_image.Add(
            self.m_statictext_right_name, 0, wx.ALIGN_CENTER | wx.ALL, 5)
        self.m_layout_right_image.Add(self.m_bitmap_right, 5, wx.ALIGN_CENTER_HORIZONTAL, 5)

        self.m_layout_stereo_image_view.Add(
            self.m_layout_left_image, 1, wx.EXPAND, 5)
        self.m_layout_stereo_image_view.Add(
            self.m_layout_right_image, 1, wx.EXPAND, 5)

        # .. sub image view layout:v X 2
        m_layout_main_view.Add(self.m_layout_tree, 1, wx.EXPAND, 5)
        m_layout_main_view.Add(
            self.m_layout_stereo_image_view, 4, wx.EXPAND, 5)
        return m_layout_main_view

    def _init_checkerboard_loader(self, parent):

        pass

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
        pass 

class StereoFileLoader(wx.Frame):
    def __init__(self, parent):
        wx.Frame.__init__(self, parent, id=wx.ID_ANY, size=wx.Size(600,200))
        self.SetMinSize(self.GetSize())
        self.SetMaxSize(self.GetSize())

        # init ui
        self.m_layout_main = wx.BoxSizer(wx.VERTICAL)
        # .
        self.m_layout_left_file = wx.BoxSizer(wx.HORIZONTAL)
        self.m_layout_right_file = wx.BoxSizer(wx.HORIZONTAL)
        self.m_layout_pattern_parameter = wx.BoxSizer(wx.HORIZONTAL)

        self.m_statictext_left_label = wx.StaticText( self, wx.ID_ANY, u"left", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_layout_main.Add(self.m_statictext_left_label)
        self.m_textctl_left_path = wx.TextCtrl(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_textctl_left_path.Enable(False)

        self.m_layout_main.Add(self.m_layout_left_file)


        self.m_statictext_right_label = wx.StaticText( self, wx.ID_ANY, u"right", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_layout_main.Add(self.m_statictext_right_label)
        self.m_textctl_right_path = wx.TextCtrl(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_textctl_right_path.Enable(False)

        self.m_layout_main.Add(self.m_layout_right_file)

        self.m_staticline = wx.StaticLine(self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
        self.m_layout_main.Add(self.m_staticline)

        self.m_layout_main.Add(self.m_layout_pattern_parameter)

        self.m_staticline2 = wx.StaticLine(self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL )
        self.m_layout_main.Add(self.m_staticline2)

        m_layout_dlgbtnsizer = wx.StdDialogButtonSizer()
        self.m_sdbOK = wx.Button( self, wx.ID_OK )
        m_layout_dlgbtnsizer.AddButton( self.m_sdbOK )
        self.m_sdbCancel = wx.Button( self, wx.ID_CANCEL )
        m_layout_dlgbtnsizer.AddButton( self.m_sdbCancel )
        self.m_layout_main.Add(m_layout_dlgbtnsizer,1,wx.EXPAND,5)


