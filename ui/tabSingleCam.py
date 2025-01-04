"""
Author: zhijian li (lzjcyy@qq.com)
tabSingleCam.py (c) 2024
Desc: mono camera calibration
Created:  2024-06-24T05:48:52.647Z
Modified: !date!
"""


import os
import sys
import threading
import cv2
import numpy as np
import json
from loguru import logger
import pickle

from utils.ophelper import *
from utils.storage import LocalStorage
from utils.calib import CalibBoard, quat_2_rot, rot_2_quat
from utils.err import CalibErrType
from ui.components import * # wx is already imported through components

IMAGE_VIEW_W = 800
IMAGE_VIEW_H = 600

Dlg_Evt_Custom = wx.NewEventType()
EVT_DLG_CUSTOM = wx.PyEventBinder(Dlg_Evt_Custom)

class TabSingleCam():
    def __init__(self, parent, tab):
        self.tab = tab
        # global var
        self.monocheck = None
        self.image_shape = None
        self.current_root_dir = None
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
        self.m_textCtrl_file_path = wx.TextCtrl(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_textCtrl_file_path.Enable(False)

        self.path_h_sizer.Add(
            self.m_textCtrl_file_path, 5, wx.ALIGN_CENTER_HORIZONTAL | wx.ALIGN_CENTER_VERTICAL | wx.ALL, 5)

        self.m_select_file_path = wx.Button(
            self.tab, wx.ID_ANY, u"Select folder", wx.DefaultPosition, wx.DefaultSize, 0)
        self.path_h_sizer.Add(self.m_select_file_path, 1,
                              wx.ALIGN_CENTER_HORIZONTAL | wx.ALIGN_CENTER_VERTICAL | wx.ALL, 5)

        sizer.Add(self.path_h_sizer, 1, wx.EXPAND, 5)

        # 组建标定板pattern设置区域
        '''
        |标定板行数|__|标定板列数|__|标定板单元格边长(mm)|__|开始标定|保存标定结果|
        '''
        pattern_border = 1
        self.m_staticText_row = wx.StaticText(
            self.tab, wx.ID_ANY, u"Number of Rows", wx.DefaultPosition, wx.DefaultSize, 0)
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
            self.tab, wx.ID_ANY, u"Number of Cols", wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_staticText_col.Wrap(-1)

        self.checkerpattern_h_sizer.Add(
            self.m_staticText_col, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, pattern_border)

        self.m_textCtrl_col = wx.TextCtrl(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, text_size, 0)
        self.checkerpattern_h_sizer.Add(
            self.m_textCtrl_col, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, pattern_border)

        self.m_staticText_cellsize = wx.StaticText(
            self.tab, wx.ID_ANY, u"Checkerboard Cell Size(mm)", wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_staticText_cellsize.Wrap(-1)

        self.checkerpattern_h_sizer.Add(
            self.m_staticText_cellsize, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, pattern_border)

        self.m_textCtrl_cellsize = wx.TextCtrl(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, text_size, 0)
        self.checkerpattern_h_sizer.Add(
            self.m_textCtrl_cellsize, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, pattern_border)

        self.m_calibrate_btn = wx.Button(
            self.tab, wx.ID_ANY, u"Calib", wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_calibrate_btn.SetBackgroundColour(wx.Colour(128, 255, 0))
        self.m_calibrate_btn.Enable(False)
        self.checkerpattern_h_sizer.Add(
            self.m_calibrate_btn, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, pattern_border)

        self.m_save_calibration_btn = wx.Button(
            self.tab, wx.ID_ANY, u"Save result", wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_save_calibration_btn.Enable(False)
        self.checkerpattern_h_sizer.Add(
            self.m_save_calibration_btn, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, pattern_border)
        
        self.m_show_pts_dist_btn = wx.Button(
            self.tab, wx.ID_ANY, u"Display the chessboard pattern distribution", wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_show_pts_dist_btn.Enable(False)
        self.checkerpattern_h_sizer.Add(
            self.m_show_pts_dist_btn, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, pattern_border
        )

        self.m_staticText_warning = wx.StaticText(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        self.checkerpattern_h_sizer.Add(
            self.m_staticText_warning, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, pattern_border)

        # add use libcbdetect 
        self.m_checkbox_use_libcbdetect = wx.CheckBox(self.tab, wx.ID_ANY, label="Use Libcbdetect")
        self.checkerpattern_h_sizer.Add(self.m_checkbox_use_libcbdetect, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, pattern_border)
        
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
        self.m_main_image_view = ImagePanel(
            self.tab, wx.Size(IMAGE_VIEW_W, IMAGE_VIEW_H))

        self.main_h_sizer.Add(self.m_main_image_view, 3,
                              wx.ALIGN_CENTER_VERTICAL, 5)

        # vtk panel
        # camera poses
        # if sys.platform != "darwin":
        #     from ui.vtkpanel import VTKPanel
        #     self.camera_pose_view = VTKPanel(self.tab, wx.Size(200, 200))
        #     self.main_h_sizer.Add(self.camera_pose_view, 1,
        #                           wx.ALIGN_CENTER_VERTICAL, 5)
        # # TODO

        # register callback
        self._register_all_callbacks()

    def init_db(self):
        # db init
        # create a single camera calib table
        '''
        use quaternion and position to represent rotation and translation
        |id integer|rootpath text|filename text|isreject bool|qw float |qx float  |qy float  |qz float  |tx float|ty float| tz float|  rpje| cors blob |
        |----------|-------------|-------------|-------------|---------|----------|----------|----------|--------|--------|---------|------|-----------|
        |    0     |c:\data\     |    img1.png |  False      |0.1085443|-0.2130855|-0.9618053|-0.1332042| -44.071| 272.898|-1388.602|0.1826|array bytes|
        |    1     |c:\data\     |    img2.png |  True       |         |          |          |          |        |        |         |      |           |
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
                            rpje float,
                            cors blob'''
        self.DB_FILENAME = ':memory:'
        self.DB_TABLENAME = 'single'
        db = LocalStorage(self.DB_FILENAME)
        ret = db.create_table(self.DB_TABLENAME, TABLE_SQL_STR)
        if ret is not True:
            logger.debug(f'create db table {self.DB_TABLENAME} failed')
            return None
        else:
            return db

    def _register_all_callbacks(self):
        self.tab.Bind(wx.EVT_BUTTON, self.on_select_file_path,
                      self.m_select_file_path)
        self.tab.Bind(wx.EVT_BUTTON, self.on_click_calibrate,
                      self.m_calibrate_btn)
        self.tab.Bind(wx.EVT_TREE_SEL_CHANGING,
                      self.on_tree_item_select, self.m_treeCtl_images)
        self.tab.Bind(wx.EVT_TREE_ITEM_RIGHT_CLICK,
                      self.on_tree_item_right_click, self.m_treeCtl_images)
        self.tab.Bind(wx.EVT_BUTTON, self.on_save_calibration_results,
                      self.m_save_calibration_btn)
        self.tab.Bind(wx.EVT_BUTTON, self.on_show_disp_details, self.m_show_pts_dist_btn)
        # text changed evt
        self.m_textCtrl_row.Bind(wx.EVT_TEXT, self.on_text_changed)
        self.m_textCtrl_col.Bind(wx.EVT_TEXT, self.on_text_changed)
        self.m_textCtrl_cellsize.Bind(wx.EVT_TEXT, self.on_text_changed)
        # dlg details updates
        self.tab.Bind(EVT_DLG_CUSTOM, self.on_dlg_details_changed)

    def _list_images_with_suffix(self, rootpath: str, suffix_list: list = ['png', 'jpg', 'jpeg', 'bmp']):
        images = []
        for f in os.listdir(rootpath):
            # on macos, listdir will create a hidden file which name starts with '.', it can not be opened by opencv
            if not f.startswith('.'):
                suffix = f.rsplit('.', 1)[-1].lower()
                if suffix in suffix_list:
                    images.append(f)
        images.sort()
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

        # 判定是否为最大误差值，并标红
        if rpjes[0] is not None:
            max_err = max(rpjes)
        else:
            max_err = None
        max_high_count = 0

        dirroot = tree.AddRoot('Filename: (Reprojection Error)', image=0)
        if len(filelist) > 0:
            for fname, r in zip(filelist, rpjes):
                newItem = tree.AppendItem(
                    dirroot, f'{fname}:({str(r)})', data=[fname, r])
                if max_err is not None:
                    if max_err == r and max_high_count < 1:
                        max_high_count += 1
                        tree.SetItemTextColour(newItem, wx.RED)
                tree.SetItemImage(newItem, self.icon_ok)
            tree.Expand(dirroot)
            tree.SelectItem(newItem)
            tree.EnsureVisible(newItem)
            tree.EnableVisibleFocus(True)

    # 更新按钮状态
    def _update_btns(self):
        path_p = self.m_textCtrl_file_path.GetValue()
        col_p = self.m_textCtrl_col.GetValue()
        row_p = self.m_textCtrl_row.GetValue()
        cell_p = self.m_textCtrl_cellsize.GetValue()

        if len(col_p) > 0 and len(row_p) > 0 and len(path_p):
            self.m_calibrate_btn.Enable()
        else:
            self.m_calibrate_btn.Enable(False)
            self.m_save_calibration_btn.Enable(False)

    # 更新图片处理进度
    def on_dlg_details_changed(self, evt):
        data = evt.get_data()

    # 输入框变动关联更新btn
    def on_text_changed(self, evt):
        self._update_btns()

    # 当左键点击图像列表中的item时，触发此处理
    def on_tree_item_select(self, evt):
        id = evt.GetItem()
        rootid = self.m_treeCtl_images.GetRootItem()
        if rootid != id:
            fullname = self.m_treeCtl_images.GetItemText(id)
            filename = fullname.split(':')[0]
            status = fullname.split(':')[1]
            image_data = cv2.imread(os.path.join(
                self.current_root_dir, filename))
            # 确定图像缩放比例，以便适配显示窗口
            img_w = image_data.shape[1]
            img_h = image_data.shape[0]
            SCALE_RATIO = img_w/IMAGE_VIEW_W
            # draw corners
            if status != '(None)':
                row = int(self.m_textCtrl_row.GetValue())
                col = int(self.m_textCtrl_col.GetValue())
                # 读取单元格边长，如果留空，默认值为 1.0 mm
                if len(self.m_textCtrl_cellsize.GetValue()) == 0:
                    cellsize = 1.0
                else:
                    cellsize = float(self.m_textCtrl_cellsize.GetValue())
                # todo 
                # retrive db's cors
                calib_instance = CalibBoard(row, col, cellsize, use_mt=False)
                result = self.db.retrive_data(self.DB_TABLENAME, "cors", f'WHERE filename=\'{filename}\' ')
                _cors = [c[0] for c in result]
                cors = pickle.loads(_cors[0])
                # _, cors = calib_instance.find_corners(
                #      cv2.cvtColor(image_data, cv2.COLOR_BGR2GRAY))
                calib_instance.draw_corners(image_data, cors)

            image_data = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
            image_data = cv2.resize(
                image_data, (int(img_w/SCALE_RATIO), int(img_h/SCALE_RATIO)))
            self.m_main_image_view.set_cvmat(image_data)
        else:
            self.m_main_image_view.set_bitmap(
                wx.Bitmap(IMAGE_VIEW_W, IMAGE_VIEW_H))

        self.m_main_image_view.Refresh()

    # 当右键点击图像列表中的item时，触发此处理
    def on_tree_item_right_click(self, evt):
        item = evt.GetItem()
        rootid = self.m_treeCtl_images.GetRootItem()
        if rootid != item:
            fname, rpje = self.m_treeCtl_images.GetItemData(item)
            menu = wx.Menu()
            itm = menu.Append(wx.ID_ANY, "Delete and recalibrate")
            self._temp_right_menu_data = fname
            self.tab.Bind(wx.EVT_MENU, self.on_recalib, itm)
            self.m_treeCtl_images.PopupMenu(menu, evt.GetPoint())
            menu.Destroy()

    # popup menu
    def on_recalib(self, evt):
        # set selected image to be rejected in db
        self.db.modify_data(self.DB_TABLENAME,
                            f'''SET isreject=1 WHERE filename=\'{self._temp_right_menu_data}\' ''')
        right_click_evt = wx.CommandEvent(
            wx.EVT_BUTTON.typeId, self.m_calibrate_btn.GetId())
        self.m_calibrate_btn.GetEventHandler().ProcessEvent(right_click_evt)

    # 加载图片文件
    def on_select_file_path(self, evt):
        dir_dialog = wx.DirDialog(
            None, "Select calibration board image path", style=wx.DD_DEFAULT_STYLE | wx.DD_NEW_DIR_BUTTON)
        if dir_dialog.ShowModal() == wx.ID_OK:
            self.current_root_dir = dir_dialog.GetPath()
            self.m_textCtrl_file_path.SetValue(self.current_root_dir)
            self.m_calibrate_btn.Enable(False)
            self.m_save_calibration_btn.Enable(False)
            self.m_show_pts_dist_btn.Enable(False)
        else:
            return
        dir_dialog.Destroy()

        images = self._list_images_with_suffix(self.current_root_dir)
        # check if there is any images
        if len(images) > 0:
            self.m_calibrate_btn.Enable(True)
            self.m_staticText_warning.SetLabel(wx.EmptyString)
            self.m_staticText_warning.ClearBackground()
        else:
            self.m_calibrate_btn.Enable(False)
            self.m_staticText_warning.SetLabel(
                "No image files found in the calibration board image path")
            self.m_staticText_warning.SetBackgroundColour(wx.Colour(255, 0, 0))

        # progress
        keep_going = True
        count = 0
        max_value = len(images)
        dlg = wx.ProgressDialog("Loading images",
                                "Loading，please wait",
                                maximum=max_value,
                                parent=self.tab,
                                style=wx.PD_APP_MODAL | wx.PD_AUTO_HIDE)
        # 删除旧记录
        self.db.delete_data(self.DB_TABLENAME, f'WHERE 1=1')
        # 写入新纪录
        for item in images:
            count += 1
            self.db.write_data(
                self.DB_TABLENAME, f'null, \'{self.current_root_dir}\', \'{item}\', 0, null, null, null, null, null, null, null, null, null')
            (keep_going, skip) = dlg.Update(count, f'added {count} images')
        # wx.Sleep(1)
        dlg.Destroy()
        # update tree ctrl
        self.update_treectrl()

    # 执行标定操作
    def on_click_calibrate(self, evt):
        if (len(self.m_textCtrl_row.GetValue()) and len(self.m_textCtrl_col.GetValue())) == 0:
            self.m_staticText_warning.SetLabel(
                "Please fill in the number of rows and columns of the calibration board first. The side length of the calibration board cells can be ignored. If precise translation is needed, then please enter the correct values.")
            self.m_staticText_warning.SetBackgroundColour(wx.Colour(255, 0, 0))
        else:
            self.m_staticText_warning.SetLabel(wx.EmptyString)
            self.m_staticText_warning.ClearBackground()

            # do calibration
            # 读取数据库中的文件列表
            results = self.db.retrive_data(
                self.DB_TABLENAME, f'rootpath, filename', f'WHERE isreject=0')
            filelist = [f[1] for f in results]
            # 读取标定板行列数
            row = int(self.m_textCtrl_row.GetValue())
            col = int(self.m_textCtrl_col.GetValue())
            # 读取单元格边长，如果留空，默认值为 1.0 mm
            if len(self.m_textCtrl_cellsize.GetValue()) == 0:
                cellsize = 1.0
            else:
                cellsize = float(self.m_textCtrl_cellsize.GetValue())

            dlg = wx.ProgressDialog(
                "Calibration", "Calibrating ...", maximum=3, parent=self.tab, style=wx.PD_APP_MODAL | wx.PD_AUTO_HIDE)
            dlg.Update(0, "Calculating ...")
            thread = threading.Thread(target=self._run_camera_calibration_task, args=(
                row, col, cellsize, results, filelist, dlg))
            thread.start()
            # test custom evt
            event = CustomEvent(Dlg_Evt_Custom)
            event.set_data('start a process thread')
            #self.tab.GetEventHandler().ProcessEvent(event)
            wx.PostEvent(self.tab, event)

    # 保存校准结果
    def on_save_calibration_results(self, evt):
        dlg = wx.FileDialog(self.tab, u"Save result", wildcard='*.json',
                            defaultFile='camera_parameters', style=wx.FD_SAVE)
        if dlg.ShowModal() == wx.ID_OK:
            path = dlg.GetPath()
            # save code here
            self._write_2_file(path, self.mtx, self.dist)
            # 打开当前保存的路径，方便用户查看
            open_folder(path)
        dlg.Destroy()
    
    # 显示拍摄分布
    def on_show_disp_details(self, evt):
        dpanel = DetailsImagePanel(self.tab.GetParent().GetParent(), "Corner distribution")
        dpanel.commit_cvdata(self.monocheck)
        dpanel.Show()
        self.tab.GetParent().GetParent().Disable()

    def _write_2_file(self, filename, mtx, dist):
        paramJsonStr = {
            'version': '0.1',
            'SN': '',
            'Scheme': 'opencv',
            'ImageShape':[self.image_shape[0], self.image_shape[1]],
            'CameraParameters': {
                'RadialDistortion': [dist.tolist()[0][0], dist.tolist()[0][1], dist.tolist()[0][-1]],
                'TangentialDistortion': [dist.tolist()[0][2], dist.tolist()[0][3]],
                'IntrinsicMatrix': mtx.tolist()
            },
            'ReprojectionError': self.rpjerr
        }
        with open(f'{filename}', 'w') as f:
            json.dump(paramJsonStr, f, indent=4)
        pass

    # 相机校准线程
    def _run_camera_calibration_task(self, row, col, cellsize, results, filelist, dlg):
        # 创建单目校准类
        calib = CalibBoard(row, col, cellsize, use_libcbdet=self.m_checkbox_use_libcbdetect.GetValue())
        CALIB = calib.mono_calib_parallel if calib.USE_MT is True else calib.mono_calib
        # 执行校准，并得到结果
        ret, mtx, dist, rvecs, tvecs, rpjes, rej_list, cal_list, shape, pts, err = CALIB(
            results[0][0], filelist)
        
        # 检查ret是否为false
        if ret is False:
            wx.CallAfter(self._camera_calibration_task_done, dlg, ret,
                         mtx, dist, rvecs, tvecs, rpjes, rej_list, cal_list, err, None, None)
            return
        self.image_shape = shape
        # draw all pts for double check
        ## calculate rpj
        RPJS=[]
        for i in range(len(rvecs)):
            rpjs, _ = cv2.projectPoints(calib.objp, np.asarray(rvecs[i]).reshape(-1,3), np.asarray(tvecs[i]).reshape(-1,3), mtx, dist)
            RPJS.append(rpjs)
        RPJS = np.asarray(RPJS).reshape(-1,2)

        img_for_dist_check = np.zeros((shape[1], shape[0], 3), dtype=np.uint8)
        pts = np.asarray(pts).reshape(-1,2)
        calib.draw_arrows(img_for_dist_check, pts, RPJS)
        #calib.draw_corners(img_for_dist_check, pts, False)
        self.monocheck = img_for_dist_check

        wx.CallAfter(self._camera_calibration_task_done, dlg, ret,
                     mtx, dist, rvecs, tvecs, rpjes, rej_list, cal_list, err, pts, RPJS)

    def _camera_calibration_task_done(self, dlg, ret, mtx, dist, rvecs, tvecs, rpjes, rej_list, cal_list, err, pts, RPJS):
        dlg.Update(1, "Finished")
        if ret is False:
            dlg.Destroy()
            # 使用wxpython创建一个msg box，并提示用户"标定失败"
            wx.MessageBox(f"Calibration Failed:{CalibErrType.to_string(err)}", "Notice", wx.OK | wx.ICON_ERROR)
            self.m_save_calibration_btn.Enable(False)
            self.m_show_pts_dist_btn.Enable(False)
            return
        # wx.Sleep(1)
        self.rpjerr = ret
        self.mtx = mtx
        self.dist = dist
        # update the database
        dlg.Update(2, "Updating information of files with failed calibration...")
        self._set_rejected_flags(rej_list)
        dlg.Update(3, "Saving calibration results to the database...")
        self._save_each_image_rt_rpje(rvecs, tvecs, rpjes, cal_list, pts, RPJS)
        wx.Sleep(1)
        dlg.Destroy()
        # update tree ctrl
        self.update_treectrl()
        # enalbe save
        self.m_save_calibration_btn.Enable(True)
        self.m_show_pts_dist_btn.Enable(True)

    # 把无法找到角点的图片列表写入数据库
    def _set_rejected_flags(self, filelist):
        for f in filelist:
            self.db.modify_data(self.DB_TABLENAME,
                                f'''SET isreject=1 WHERE filename=\'{f}\' ''')

    # 把标定结果写入数据库
    def _save_each_image_rt_rpje(self, rvecs, tvecs, rpjes, filelist, pts, RPJS):
        if len(rvecs) == len(filelist):
            # devide pts/RPJS into each image by len(filelist)
            pts_split = np.split(pts, len(filelist))
            RPJS_split = np.split(RPJS, len(filelist))

            for f, rv, tv, rpje, _pts, _RPJS in zip(filelist, rvecs, tvecs, rpjes, pts_split, RPJS_split):
                # convert rt vecs into quat
                R, _ = cv2.Rodrigues(rv)
                q = rot_2_quat(R)
                rpje = "{:.3f}".format(float(rpje))
                # cors to blob
                cors_bytes = pickle.dumps(_pts)
                self.db.modify_data(
                    self.DB_TABLENAME, f'''SET isreject=0, 
                                        qw={float(q[0])}, 
                                        qx={float(q[1])},
                                        qy={float(q[2])},
                                        qz={float(q[3])},
                                        tx={float(tv[0])},
                                        ty={float(tv[1])},
                                        tz={float(tv[2])},
                                        rpje={float(rpje)},
                                        cors=?
                                        WHERE filename=\'{f}\' ''', (cors_bytes,))
        else:
            logger.debug(f'please check the file list')
