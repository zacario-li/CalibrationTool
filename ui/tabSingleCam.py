import cv2
import wx
import os
from pathlib import Path
from utils.storage import LocalStorage
from utils.calib import CalibChessboard, quat2rot, rot2quat
from loguru import logger


class TabSingleCam():
    def __init__(self, parent, tab):
        self.tab = tab
        # global var
        self.current_dir = None
        self.db = self.init_db()
        # intrinsic and distortion coef, also reprojection error
        self.mtx = None
        self.dist = None
        self.rpjerr = None
        # checkerboard's pattern
        self.checkerboard_row_cell = 0
        self.checkerboard_col_cell = 0
        self.checkerboard_cell_size_in_mm = 0.0

        # init ui
        sizer = wx.BoxSizer(wx.VERTICAL)
        self.tab.SetSizer(sizer)
        # 文件夹选择框layout
        self.path_h_sizer = wx.BoxSizer(wx.HORIZONTAL)
        # 标定板pattern设置layout
        self.checkerpattern_h_sizer = wx.BoxSizer(wx.HORIZONTAL)
        # 主显示区域的水平layout
        self.main_h_sizer = wx.BoxSizer(wx.HORIZONTAL)

        # 组建文件夹选择区域
        '''
        |文件路径|选择按钮|
        '''
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

        # 组建标定板pattern设置区域
        '''
        |标定板行数|__|标定板列数|__|标定板单元格边长(mm)|__|开始标定|
        '''
        pattern_border = 1
        self.m_staticText_row = wx.StaticText(
            self.tab, wx.ID_ANY, u"标定板行数", wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_staticText_row.Wrap(-1)
        # 获取上述控件大小，便于设定 textCtrl 大小
        text_size = self.m_staticText_row.GetSize()

        self.checkerpattern_h_sizer.Add(
            self.m_staticText_row, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, pattern_border)

        self.m_textCtrl_row = wx.TextCtrl(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, text_size, 0)
        self.checkerpattern_h_sizer.Add(
            self.m_textCtrl_row, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, pattern_border)

        self.m_staticText_col = wx.StaticText(
            self.tab, wx.ID_ANY, u"标定板列数", wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_staticText_col.Wrap(-1)

        self.checkerpattern_h_sizer.Add(
            self.m_staticText_col, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, pattern_border)

        self.m_textCtrl_col = wx.TextCtrl(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, text_size, 0)
        self.checkerpattern_h_sizer.Add(
            self.m_textCtrl_col, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, pattern_border)

        self.m_staticText_cellsize = wx.StaticText(
            self.tab, wx.ID_ANY, u"标定板单元格边长(mm)", wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_staticText_cellsize.Wrap(-1)

        self.checkerpattern_h_sizer.Add(
            self.m_staticText_cellsize, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, pattern_border)

        self.m_textCtrl_cellsize = wx.TextCtrl(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, text_size, 0)
        self.checkerpattern_h_sizer.Add(
            self.m_textCtrl_cellsize, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, pattern_border)

        self.m_calibrate_btn = wx.Button(
            self.tab, wx.ID_ANY, u"开始标定", wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_calibrate_btn.SetBackgroundColour(wx.Colour(128, 255, 0))
        self.m_calibrate_btn.Enable(False)
        self.checkerpattern_h_sizer.Add(
            self.m_calibrate_btn, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, pattern_border)

        self.m_staticText_warning = wx.StaticText(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        self.checkerpattern_h_sizer.Add(
            self.m_staticText_warning, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, pattern_border)

        sizer.Add(self.checkerpattern_h_sizer, 1, wx.ALL, 5)

        # 分隔线
        self.m_staticline1 = wx.StaticLine(
            self.tab, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL)
        sizer.Add(self.m_staticline1, 0, wx.EXPAND | wx.ALL, 5)

        # 进入主显示区域
        '''
        |image tree|image view|camera poses|
        '''
        sizer.Add(self.main_h_sizer, 20, wx.ALL, 5)

        # tree ctrl
        self.iconlist = wx.ImageList(16, 16)
        self.icon_ok = self.iconlist.Add(wx.ArtProvider.GetBitmap(
            wx.ART_INFORMATION, wx.ART_OTHER, (16, 16)))
        self.icon_q = self.iconlist.Add(wx.ArtProvider.GetBitmap(
            wx.ART_CROSS_MARK, wx.ART_OTHER, (16, 16)))
        self.m_treeCtl_images = self.create_treectrl()

        self.main_h_sizer.Add(self.m_treeCtl_images, 1, wx.EXPAND, 5)

        # image view
        self.m_main_image_view = wx.StaticBitmap(
            self.tab, size=wx.Size(800, 600))
        self.m_main_image_view.SetScaleMode(wx.StaticBitmap.Scale_AspectFill)

        self.main_h_sizer.Add(self.m_main_image_view, 3, wx.EXPAND, 5)

        # camera poses
        # TODO

        # register callback
        self.tab.Bind(wx.EVT_BUTTON, self.on_select_file_path,
                      self.m_select_file_path)
        self.tab.Bind(wx.EVT_BUTTON, self.on_click_calibrate,
                      self.m_calibrate_btn)
        self.tab.Bind(wx.EVT_TREE_SEL_CHANGING,
                      self.on_tree_item_select, self.m_treeCtl_images)
        self.tab.Bind(wx.EVT_TREE_ITEM_RIGHT_CLICK,
                      self.on_tree_item_right_click, self.m_treeCtl_images)
        self.tab.Bind(wx.EVT_PAINT, self.on_paint_main_imageview)

    def init_db(self):
        # db init
        # create a single camera calib table
        '''
        use quaternion and position to represent rotation and translation
        |id integer|rootpath text|filename text|isreject bool|qw float |qx float  |qy float  |qz float  |tx float|ty float| tz float|  rpje|
        |----------|-------------|-------------|-------------|---------|----------|----------|----------|--------|--------|---------|------|
        |    0     |c:\data\     |    img1.png |  False      |0.1085443|-0.2130855|-0.9618053|-0.1332042| -44.071| 272.898|-1388.602|0.1826|
        |    1     |c:\data\     |    img2.png |  True       |         |          |          |          |        |        |         |      |
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
                            tz float,
                            rpje float'''
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
            # on macos, listdir will create a hidden file which name starts with '.', it can not be opened by opencv
            if not f.startswith('.'):
                suffix = f.rsplit('.', 1)[-1].lower()
                if suffix in suffix_list:
                    images.append(f)
        return images

    # 左侧树形目录显示每张标定结果的控件
    def create_treectrl(self):
        tree = wx.TreeCtrl(self.tab, size=wx.Size(250, -1))
        tree.AssignImageList(self.iconlist)
        return tree

    # 更新目录条目信息
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
        tree.SelectItem(dirroot)
        if len(filelist) > 0:
            for fname, r in zip(filelist, rpjes):
                newItem = tree.AppendItem(dirroot, f'{fname}:({str(r)})')
                tree.SetItemImage(newItem, self.icon_ok)
            tree.Expand(dirroot)

    def on_tree_item_select(self, evt):
        self.m_main_image_view.Refresh()

    def on_tree_item_right_click(self, evt):
        self.m_main_image_view.Refresh()
        item = evt.GetItem()

    def on_paint_main_imageview(self, evt):
        dc = wx.BufferedPaintDC(self.m_main_image_view)
        pass

    # 加载图片文件
    def on_select_file_path(self, evt):
        dir_dialog = wx.DirDialog(
            None, "选择校准图像路径", style=wx.DD_DEFAULT_STYLE | wx.DD_NEW_DIR_BUTTON)
        if dir_dialog.ShowModal() == wx.ID_OK:
            self.current_dir = dir_dialog.GetPath()
            self.m_textCtrl1.SetValue(self.current_dir)
            self.m_calibrate_btn.Enable(False)
        else:
            return
        dir_dialog.Destroy()

        images = self.list_images_with_suffix(self.current_dir)
        # check if there is any images
        if len(images) > 0:
            self.m_calibrate_btn.Enable(True)
            self.m_staticText_warning.SetLabel(wx.EmptyString)
            self.m_staticText_warning.ClearBackground()
        else:
            self.m_calibrate_btn.Enable(False)
            self.m_staticText_warning.SetLabel(
                "！！！！！！！！！！！未找到图片！！！！！！！！！！！")
            self.m_staticText_warning.SetBackgroundColour(wx.Colour(255, 0, 0))

        # progress
        keep_going = True
        count = 0
        max_value = len(images)
        dlg = wx.ProgressDialog("加载图像",
                                "图片加载中，请稍后",
                                maximum=max_value,
                                parent=self.tab,
                                style=wx.PD_APP_MODAL | wx.PD_AUTO_HIDE)
        # 删除旧记录
        self.db.delete_data(self.DB_TABLENAME, f'WHERE 1=1')
        # 写入新纪录
        for item in images:
            count += 1
            self.db.write_data(
                self.DB_TABLENAME, f'null, \'{self.current_dir}\', \'{item}\', 0, null, null, null, null, null, null, null, null')
            (keep_going, skip) = dlg.Update(count, f'added {count} images')
        wx.Sleep(1)
        dlg.Destroy()
        # update tree ctrl
        self.update_treectrl()

    # 执行标定操作
    def on_click_calibrate(self, evt):
        if (len(self.m_textCtrl_row.GetValue()) and len(self.m_textCtrl_col.GetValue())) == 0:
            self.m_staticText_warning.SetLabel(
                "请先填写标定板的行和列数，标定板单元格边长可以忽略，如果需要精确的 translation，那么请输入正确的值")
            self.m_staticText_warning.SetBackgroundColour(wx.Colour(255, 0, 0))
        else:
            self.m_staticText_warning.SetLabel(wx.EmptyString)
            self.m_staticText_warning.ClearBackground()

            # do calibration
            # 读取数据库中的文件列表
            results = self.db.retrive_data(
                self.DB_TABLENAME, f'rootpath, filename')
            filelist = [f[1] for f in results]
            # 读取标定板行列数
            row = int(self.m_textCtrl_row.GetValue())
            col = int(self.m_textCtrl_col.GetValue())
            # 读取单元格边长，如果留空，默认值为 1.0 mm
            if len(self.m_textCtrl_cellsize.GetValue()) == 0:
                cellsize = 1.0
            else:
                cellsize = float(self.m_textCtrl_cellsize.GetValue())

            # 创建单目校准类
            calib = CalibChessboard(row, col, cellsize)
            dlg = wx.ProgressDialog(
                "标定", "正在标定...", maximum=3, parent=self.tab, style=wx.PD_APP_MODAL | wx.PD_AUTO_HIDE)
            dlg.Update(0, "开始计算")
            # 执行校准，并得到结果
            ret, mtx, dist, rvecs, tvecs, rpjes, rej_list, cal_list = calib.single_calib(
                results[0][0], filelist)
            dlg.Update(1, "计算结束")
            wx.Sleep(1)
            self.rpjerr = ret
            self.mtx = mtx
            self.dist = dist
            # update the database
            dlg.Update(2, "更新校准失败的文件信息...")
            self._set_rejected_flags(rej_list)
            dlg.Update(3, "保存标定结果到数据库...")
            self._save_each_image_rt_rpje(rvecs, tvecs, rpjes, cal_list)
            wx.Sleep(1)
            dlg.Destroy()
            # update tree ctrl
            self.update_treectrl()

    def _set_rejected_flags(self, filelist):
        for f in filelist:
            self.db.modify_data(self.DB_TABLENAME,
                                f'''SET isreject=1 WHERE filename=\'{f}\' ''')

    def _save_each_image_rt_rpje(self, rvecs, tvecs, rpjes, filelist):
        if len(rvecs) == len(filelist):
            for f, rv, tv, rpje in zip(filelist, rvecs, tvecs, rpjes):
                # convert rt vecs into quat
                R, _ = cv2.Rodrigues(rv)
                q = rot2quat(R)
                rpje = "{:.3f}".format(float(rpje))
                self.db.modify_data(
                    self.DB_TABLENAME, f'''SET isreject=0, 
                                        qw={float(q[0])}, 
                                        qx={float(q[1])},
                                        qy={float(q[2])},
                                        qz={float(q[3])},
                                        tx={float(tv[0])},
                                        ty={float(tv[1])},
                                        tz={float(tv[2])},
                                        rpje={float(rpje)} 
                                        WHERE filename=\'{f}\' ''')
        else:
            logger.debug(f'please check the file list')
