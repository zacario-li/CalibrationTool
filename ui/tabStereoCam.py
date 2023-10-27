import wx
import os
import cv2
import json
import threading
from utils.storage import LocalStorage
from ui.components import ImagePanel
from utils.calib import CalibChessboard, quat_2_rot, rot_2_quat
from utils.ophelper import *
from loguru import logger

IMAGE_VIEW_W = 480
IMAGE_VIEW_H = 270


class TabStereoCam():
    def __init__(self, parent, tab):
        self.tab = tab
        # global var
        self.image_shape = None
        self.db = self.init_db()
        self.current_leftroot = ''
        self.current_rightroot = ''
        self.current_row_cors = ''
        self.current_col_cors = ''
        self.current_cellsize = ''
        self.current_leftfile_list = []
        self.current_rightfile_list = []

        # calib result
        self.rpjerr = None
        self.mtx1 = None
        self.mtx2 = None
        self.dist1 = None
        self.dist2 = None
        self.R = None
        self.T = None
        self.F = None
        self.E = None
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
            wx.Size(IMAGE_VIEW_W, IMAGE_VIEW_H))
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
        self.m_btn_calibrate.Enable(False)
        m_layout_actions_btns.Add(
            self.m_btn_calibrate, 1, wx.EXPAND, 5)

        self.m_btn_save_calibration = wx.Button(
            self.tab, wx.ID_ANY, u"保存标定结果", wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_btn_save_calibration.Enable(False)
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
        self.tab.Bind(wx.EVT_BUTTON, self.on_click_calibrate,
                      self.m_btn_calibrate)
        self.tab.Bind(wx.EVT_BUTTON, self.on_save_calibration_results,
                      self.m_btn_save_calibration)
        self.tab.Bind(wx.EVT_TREE_SEL_CHANGING,
                      self.on_tree_item_select, self.m_treectrl)
        self.tab.Bind(wx.EVT_TREE_ITEM_RIGHT_CLICK,
                      self.on_tree_item_right_click, self.m_treectrl)

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
        self.DB_FILENAME = ':memory:'
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
        self.iconlist = wx.ImageList(16, 16)
        self.icon_ok = self.iconlist.Add(wx.ArtProvider.GetBitmap(
            wx.ART_INFORMATION, wx.ART_OTHER, (16, 16)))
        self.icon_q = self.iconlist.Add(wx.ArtProvider.GetBitmap(
            wx.ART_CROSS_MARK, wx.ART_OTHER, (16, 16)))
        tree = wx.TreeCtrl(self.tab)
        tree.AssignImageList(self.iconlist)
        return tree

    # 更新目录条目信息 TODO
    def update_treectrl(self, all: bool = False):
        tree = self.m_treectrl
        tree.DeleteAllItems()
        # retrive qualified images from db
        if all is False:
            left_condi = f'WHERE isreject=0 AND cameraid=0'
            right_condi = f'WHERE isreject=0 AND cameraid=1'
        else:
            left_condi = 'WHERE cameraid=0'
            right_condi = 'WHERE cameraid=1'

        left_results = self.db.retrive_data(
            self.DB_TABLENAME, f'rootpath, filename, rpje', left_condi)
        right_results = self.db.retrive_data(
            self.DB_TABLENAME, f'rootpath, filename, rpje', right_condi)
        left_filelist = [f[1] for f in left_results]
        right_filelist = [f[1] for f in right_results]
        left_rpjes = [r[2] for r in left_results]
        right_rpjes = [r[2] for r in right_results]

        # 判定是否为最大误差值，并标红
        if left_rpjes[0] is not None:
            left_max_err = max(left_rpjes)
        else:
            left_max_err = None
        if right_rpjes[0] is not None:
            right_max_err = max(right_rpjes)
        else:
            right_max_err = None
        max_high_count = 0

        dirroot = tree.AddRoot('文件名:(左/右重投影误差)', image=0)
        if len(left_filelist) > 0:
            for lfname, lr, rfname, rr in zip(left_filelist, left_rpjes, right_filelist, right_rpjes):
                newItem = tree.AppendItem(
                    dirroot, f'{lfname},{rfname}:({str(lr)},{str(rr)})', data=[lfname, rfname])
                if left_max_err is not None and right_max_err is not None:
                    if (left_max_err == lr or right_max_err == rr) and max_high_count < 2:
                        max_high_count += 1
                        tree.SetItemTextColour(newItem, wx.RED)
                tree.SetItemImage(newItem, self.icon_ok)
            tree.Expand(dirroot)
            tree.SelectItem(newItem)
            tree.EnsureVisible(newItem)
            tree.EnableVisibleFocus(True)

    def on_tree_item_select(self, evt):
        id = evt.GetItem()
        rootid = self.m_treectrl.GetRootItem()
        if rootid != id:
            fnames = self.m_treectrl.GetItemData(id)
            fullname = self.m_treectrl.GetItemText(id)
            status = fullname.split(':')[1]
            limage_data = cv2.imread(os.path.join(
                self.current_leftroot, fnames[0]))
            rimage_data = cv2.imread(os.path.join(
                self.current_rightroot, fnames[1]))
            # 确定图像缩放比例，以便适配显示窗口
            img_w = limage_data.shape[1]
            img_h = limage_data.shape[0]
            SCALE_RATIO = img_w/IMAGE_VIEW_W
            if status != '(None,None)':
                row = int(self.current_row_cors)
                col = int(self.current_col_cors)
                cellsize = int(self.current_cellsize)
                calib_instance = CalibChessboard(row, col, cellsize)
                _, lcors = calib_instance.find_corners(
                    cv2.cvtColor(limage_data, cv2.COLOR_BGR2GRAY))
                _, rcors = calib_instance.find_corners(
                    cv2.cvtColor(rimage_data, cv2.COLOR_BGR2GRAY))
                calib_instance.draw_corners(limage_data, lcors)
                calib_instance.draw_corners(rimage_data, rcors)
            limage_data = cv2.cvtColor(limage_data, cv2.COLOR_BGR2RGB)
            rimage_data = cv2.cvtColor(rimage_data, cv2.COLOR_BGR2RGB)
            limage_data = cv2.resize(
                limage_data, (int(img_w/SCALE_RATIO), int(img_h/SCALE_RATIO)))
            rimage_data = cv2.resize(
                rimage_data, (int(img_w/SCALE_RATIO), int(img_h/SCALE_RATIO)))
            self.m_bitmap_left.set_cvmat(limage_data)
            self.m_bitmap_right.set_cvmat(rimage_data)
            self.m_statictext_left_name.SetLabel(fnames[0])
            self.m_statictext_right_name.SetLabel(fnames[1])
        else:
            self.m_bitmap_left.set_bitmap(
                wx.Bitmap(IMAGE_VIEW_W, IMAGE_VIEW_H))
            self.m_bitmap_right.set_bitmap(
                wx.Bitmap(IMAGE_VIEW_W, IMAGE_VIEW_H))
        self.m_bitmap_left.Refresh()
        self.m_bitmap_right.Refresh()

    def on_tree_item_right_click(self, evt):
        item = evt.GetItem()
        rootid = self.m_treectrl.GetRootItem()
        if rootid != item:
            lfname, rfname = self.m_treectrl.GetItemData(item)
            menu = wx.Menu()
            itm = menu.Append(wx.ID_ANY, "删除并重新标定")
            self._temp_right_menu_data = [lfname, rfname]
            self.tab.Bind(wx.EVT_MENU, self.on_recalib, itm)
            self.m_treectrl.PopupMenu(menu, evt.GetPoint())
            menu.Destroy()

    # popup menu
    def on_recalib(self, evt):
        self.db.modify_data(self.DB_TABLENAME,
                            f'''SET isreject=1 WHERE filename=\'{self._temp_right_menu_data[0]}\' ''')
        self.db.modify_data(self.DB_TABLENAME,
                            f'''SET isreject=1 WHERE filename=\'{self._temp_right_menu_data[1]}\' ''')
        right_click_evt = wx.CommandEvent(
            wx.EVT_BUTTON.typeId, self.m_btn_calibrate.GetId())
        self.m_btn_calibrate.GetEventHandler().ProcessEvent(right_click_evt)

    def on_open_file_loader(self, evt):
        dlg_file_loader = self._init_checkerboard_loader(None, self)
        dlg_file_loader.ShowModal()
        self.m_textctl_left_path.SetValue(self.current_leftroot)
        self.m_textctl_right_path.SetValue(self.current_rightroot)

    def on_click_calibrate(self, evt):
        sqlresult = self.db.retrive_data(
            self.DB_TABLENAME, f'rootpath, cameraid, filename', f'WHERE isreject=0')
        dlg = wx.ProgressDialog(
            "标定",
            "正在标定...",
            maximum=3,
            parent=self.tab,
            style=wx.PD_APP_MODAL | wx.PD_AUTO_HIDE
        )
        dlg.Update(1, "开始计算")
        thread = threading.Thread(target=self._run_camera_calibration_task,
                                  args=(dlg, sqlresult))
        thread.start()

    def on_save_calibration_results(self, evt):
        dlg = wx.FileDialog(self.tab, u"保存标定结果", wildcard='*.json',
                            defaultFile='camera_parameters', style=wx.FD_SAVE)
        if dlg.ShowModal() == wx.ID_OK:
            path = dlg.GetPath()
            # save code here
            self._write_2_file(path)
            # 打开当前保存的路径，方便用户查看
            open_folder(path)
        dlg.Destroy()

    def _write_2_file(self, filename):
        dc1 = self.dist1
        cm1 = self.mtx1
        dc2 = self.dist2
        cm2 = self.mtx2
        r = self.R
        t = self.T
        e = self.E
        f = self.F
        paramJsonStr = {
            'version': '0.1',
            'SN': '',
            'Scheme': 'opencv',
            'ImageShape':[self.image_shape[0], self.image_shape[1]],
            'CameraParameters1': {
                'RadialDistortion': [dc1.tolist()[0][0], dc1.tolist()[0][1], dc1.tolist()[0][-1]],
                'TangentialDistortion': [dc1.tolist()[0][2], dc1.tolist()[0][3]],
                'IntrinsicMatrix': cm1.tolist()
            },
            'CameraParameters2': {
                'RadialDistortion': [dc2.tolist()[0][0], dc2.tolist()[0][1], dc2.tolist()[0][-1]],
                'TangentialDistortion': [dc2.tolist()[0][2], dc2.tolist()[0][3]],
                'IntrinsicMatrix': cm2.tolist()
            },
            'RotationOfCamera2': r.tolist(),
            'TranslationOfCamera2': t.reshape(-1).tolist(),
            'FundamentalMatrix': f.tolist(),
            'EssentialMatrix': e.tolist()
        }
        with open(f'{filename}', 'w') as f:
            json.dump(paramJsonStr, f, indent=4)

    def _run_camera_calibration_task(self, dlg, sqlresult):
        results = sqlresult

        left_file_list = [f for f in results if f[1] == 0]
        right_file_list = [f for f in results if f[1] == 1]
        row = int(self.current_row_cors)
        col = int(self.current_col_cors)
        cellsize = float(self.current_cellsize)

        # 因为左右相机的图像命名相同，所以选左相机的图详列表名称
        lfilelist = [f[2] for f in left_file_list]
        rfilelist = [f[2] for f in right_file_list]

        calib = CalibChessboard(row, col, cellsize)
        CALIB = calib.stereo_calib_parallel if calib.USE_MT is True else calib.stereo_calib

        ret, mtx_l0, dist_l0, mtx_r0, dist_r0, R, T, E, F, rvecs, tvecs, pererr, rej_list, calib_list, shape = CALIB(
            left_file_list[0][0], right_file_list[0][0], lfilelist, rfilelist)
        self.image_shape = shape
        wx.CallAfter(self._camera_calibration_task_done, dlg, (ret, mtx_l0, dist_l0,
                     mtx_r0, dist_r0, R, T, E, F, rvecs, tvecs, pererr, rej_list, calib_list))

    def _camera_calibration_task_done(self, dlg, data: tuple):
        dlg.Update(2, "计算结束")
        self.rpjerr = data[0]
        self.mtx1 = data[1]
        self.dist1 = data[2]
        self.mtx2 = data[3]
        self.dist2 = data[4]
        self.R = data[5]
        self.T = data[6]
        self.E = data[7]
        self.F = data[8]
        rvecs = data[9]
        tvecs = data[10]
        pererr = data[11]
        rej_list = data[12]
        calib_list = data[13]
        dlg.Update(3, "保存标定结果到数据库...")
        self._set_rejected_flags(rej_list)
        self._save_each_image_rt_rpje((rvecs, tvecs, pererr, calib_list))
        dlg.Destroy()
        self.update_treectrl()
        self.m_btn_save_calibration.Enable()

    def _set_rejected_flags(self, rejlist: list):
        for f in rejlist:
            self.db.modify_data(self.DB_TABLENAME,
                                f'''SET isreject=1 WHERE filename=\'{f[0]}\' ''')
            self.db.modify_data(self.DB_TABLENAME,
                                f'''SET isreject=1 WHERE filename=\'{f[1]}\' ''')

    def _save_each_image_rt_rpje(self, data: tuple):
        rvecs = data[0]
        tvecs = data[1]
        pererr = data[2]
        calib_list = data[3]
        if len(rvecs) == len(calib_list):
            for f, rv, tv, rpje in zip(calib_list, rvecs, tvecs, pererr):
                R, _ = cv2.Rodrigues(rv)
                q = rot_2_quat(R)
                lrpje = "{:.3f}".format(float(rpje[0]))
                rrpje = "{:.3f}".format(float(rpje[1]))
                self.db.modify_data(
                    self.DB_TABLENAME,
                    f'''SET isreject=0,
                    qw={float(q[0])},
                    qx={float(q[1])},
                    qy={float(q[2])},
                    qz={float(q[3])},
                    tx={float(tv[0])},
                    ty={float(tv[1])},
                    tz={float(tv[2])},
                    rpje={float(lrpje)} 
                    WHERE filename=\'{f[0]}\' AND cameraid=0
                    ''')
                self.db.modify_data(
                    self.DB_TABLENAME,
                    f'''SET isreject=0,
                    qw={float(q[0])},
                    qx={float(q[1])},
                    qy={float(q[2])},
                    qz={float(q[3])},
                    tx={float(tv[0])},
                    ty={float(tv[1])},
                    tz={float(tv[2])},
                    rpje={float(rrpje)} 
                    WHERE filename=\'{f[1]}\' AND cameraid=1
                    ''')


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
        # load files
        self.pp.current_leftfile_list = self._list_images_with_suffix(lp)
        self.pp.current_rightfile_list = self._list_images_with_suffix(rp)

        if len(self.pp.current_leftfile_list) > 0 and len(self.pp.current_rightfile_list) > 0:
            self.pp.m_btn_calibrate.Enable()
        # progress
        count = 0
        max_value = len(self.pp.current_leftfile_list)
        dlg = wx.ProgressDialog("加载图像",
                                "图片加载中，请稍后",
                                maximum=max_value,
                                parent=self,
                                style=wx.PD_APP_MODAL | wx.PD_AUTO_HIDE)
        self.pp.db.delete_data(self.pp.DB_TABLENAME, f"WHERE 1=1")
        for litem, ritem in zip(self.pp.current_leftfile_list, self.pp.current_rightfile_list):
            count += 1
            self.pp.db.write_data(
                self.pp.DB_TABLENAME, f'null, \'{lp}\', 0,\'{litem}\', 0, null, null, null, null, null, null, null, null')
            self.pp.db.write_data(
                self.pp.DB_TABLENAME, f'null, \'{rp}\', 1,\'{ritem}\', 0, null, null, null, null, null, null, null, null')
            (keep_going, skip) = dlg.Update(count, f'added {count} images')
        dlg.Destroy()
        self.pp.update_treectrl()
        self.EndModal(wx.ID_OK)
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

    def _list_images_with_suffix(self, rootpath: str, suffix_list: list = ['png', 'jpg', 'jpeg', 'bmp']):
        images = []
        for f in os.listdir(rootpath):
            # on macos, listdir will create a hidden file which name starts with '.', it can not be opened by opencv
            if not f.startswith('.'):
                suffix = f.rsplit('.', 1)[-1].lower()
                if suffix in suffix_list:
                    images.append(f)
        return images

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
