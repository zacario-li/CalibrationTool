"""
Author: zhijian li (lzjcyy@qq.com)
tabStereoDisparity.py (c) 2024
Desc: support SGBM/SGM in OpenCV
Created:  2024-06-24T05:47:43.710Z
Modified: !date!
"""

import wx
import os
import cv2
import json
import threading
import numpy as np
from utils.storage import LocalStorage
from ui.components import *
from utils.calib import CalibChessboard, load_camera_param
from utils.err import CalibErrType
from utils.ophelper import *
from utils.depth import SgbmCpu
from loguru import logger
from ui.vtkpanel import VTKPanel

DISP_IMAGE_VIEW_W = 576
DISP_IMAGE_VIEW_H = 384


class TabStereoDisparity():
    def __init__(self, parent, tab):
        self.tab = tab
        self.db = self.init_db()

        # stereo parameters
        self.cam1_mtx = None
        self.cam1_dist = None
        self.cam2_mtx = None
        self.cam2_dist = None
        self.rofCam2 = None
        self.tofCam2 = None

        # default images path
        self.m_left_image_path = ''
        self.m_right_image_path = ''

        self.m_layout_main = wx.BoxSizer(wx.VERTICAL)
        self.tab.SetSizer(self.m_layout_main)
        self.init_ui()
        self._register_all_callbacks()

    def init_db(self):
        '''
        |id integer|rootpath text|cameraid int|filename text|
        |----------|-------------|------------|-------------|
        |   0      |c:\data\L    |0           |img1.png     |
        |   1      |c:\data\R    |1           |img1.png     |
        '''
        TABLE_SQL_STR = '''id INTEGER PRIMARY KEY AUTOINCREMENT,
                            rootpath text,
                            cameraid int,
                            filename text'''
        self.DB_FILENAME = ':memory:'
        self.DB_TABLENAME = 'disparity'
        db = LocalStorage(self.DB_FILENAME)
        ret = db.create_table(self.DB_TABLENAME, TABLE_SQL_STR)
        if ret is not True:
            logger.debug(f'create db table {self.DB_TABLENAME} failed')
            return None
        else:
            return db

    def init_ui(self):
        # 双目参数加载UI
        m_layout_params = self._create_ui_params()
        self.m_layout_main.Add(m_layout_params, 0, wx.EXPAND | wx.ALL, 0)
        # 按钮操作UI
        m_layout_operations = self._create_operations_ui()
        self.m_layout_main.Add(m_layout_operations, 0, wx.EXPAND | wx.ALL, 0)
        # 图像显示列表及disparity&points
        m_layout_data_view = self._create_view_ui()
        self.m_layout_main.Add(m_layout_data_view, 20, wx.EXPAND | wx.ALL, 0)

    def _create_ui_params(self):
        m_layout_params = wx.BoxSizer(wx.HORIZONTAL)

        self.m_textctrl_param_path = wx.TextCtrl(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_params.Add(self.m_textctrl_param_path, 10, wx.ALL, 1)
        self.m_textctrl_param_path.Enable(False)

        self.m_btn_load_param = wx.Button(self.tab, wx.ID_ANY, u"Load Camera Parameters",
                                          wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_params.Add(self.m_btn_load_param, 0, wx.ALL, 1)

        return m_layout_params

    def _create_operations_ui(self):
        m_layout_operations = wx.BoxSizer(wx.HORIZONTAL)

        self.m_btn_op_load_images = wx.Button(self.tab, wx.ID_ANY, u"Load Images",
                                              wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_btn_op_depth = wx.Button(self.tab, wx.ID_ANY, u"Compute Depth",
                                        wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_btn_op_rectify = wx.Button(self.tab, wx.ID_ANY, u"Rectify Images",
                                          wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_btn_op_save = wx.Button(self.tab, wx.ID_ANY, u"Save Results",
                                       wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_operations.Add(self.m_btn_op_load_images, 0, wx.ALL, 1)
        m_layout_operations.Add(self.m_btn_op_depth, 0, wx.ALL, 1)
        m_layout_operations.Add(self.m_btn_op_rectify, 0, wx.ALL, 1)
        m_layout_operations.Add(self.m_btn_op_save, 0, wx.ALL, 1)

        return m_layout_operations

    def _create_view_ui(self):
        m_layout_data_view = wx.BoxSizer(wx.HORIZONTAL)

        # image list
        self.m_treectrl = self.create_treectrl()
        m_layout_data_view.Add(self.m_treectrl, 2, wx.EXPAND | wx.ALL, 0)

        # disparity panel
        self.m_panel_image = ImagePanel(self.tab, wx.Size(576, 384))
        m_layout_data_view.Add(self.m_panel_image, 5,
                               wx.ALIGN_CENTER | wx.ALL, 5)

        # points panel
        self.m_panel_points = VTKPanel(self.tab, wx.Size(576, 384))
        m_layout_data_view.Add(self.m_panel_points, 5,
                               wx.ALIGN_CENTER | wx.ALL, 5)

        return m_layout_data_view

    def _register_all_callbacks(self):
        self.tab.Bind(wx.EVT_BUTTON, self.on_load_param_click,
                      self.m_btn_load_param)
        self.tab.Bind(wx.EVT_BUTTON, self.on_load_images_click,
                      self.m_btn_op_load_images)
        self.tab.Bind(wx.EVT_BUTTON, self.on_op_depth_click,
                      self.m_btn_op_depth)
        self.tab.Bind(wx.EVT_BUTTON, self.on_op_rectify_click,
                      self.m_btn_op_rectify)
        self.tab.Bind(wx.EVT_BUTTON, self.on_op_save_click,
                      self.m_btn_op_save)
        self.tab.Bind(wx.EVT_TREE_SEL_CHANGING,
                      self.on_tree_item_selected,
                      self.m_treectrl)
        pass

    def on_load_param_click(self, evt):
        dlg = wx.FileDialog(
            self.tab, u"Select Camera Parameters", wildcard="*.json")
        if dlg.ShowModal() == wx.ID_OK:
            filepath = dlg.GetPath()
            self.cam1_mtx, self.cam1_dist = load_camera_param(filepath)
            if not isinstance(self.cam1_mtx, np.ndarray) or not isinstance(self.cam1_dist, np.ndarray):
                wx.MessageBox(
                    f"Failed to load camera1 parameters from {filepath}!", "Error", wx.OK | wx.ICON_ERROR)
                self.m_textctrl_param_path.SetLabel("")
                return

            self.cam2_mtx, self.cam2_dist, self.rofCam2, self.tofCam2 = load_camera_param(
                filepath, camera_id=True, need_rt=True)
            if not isinstance(self.cam2_mtx, np.ndarray) or not isinstance(self.cam2_dist, np.ndarray) or not isinstance(self.rofCam2, np.ndarray) or not isinstance(self.tofCam2, np.ndarray):
                wx.MessageBox(
                    f"Failed to load camera2 parameters from {filepath}!", "Error", wx.OK | wx.ICON_ERROR)
                self.m_textctrl_param_path.SetLabel("")
                return

            self.m_textctrl_param_path.SetLabel(filepath)

        dlg.Destroy()

    def on_load_images_click(self, evt):
        dlg_filg_loader = StereoFileLoader(
            None, self.m_left_image_path, self.m_right_image_path)
        if dlg_filg_loader.ShowModal() == wx.ID_OK:
            self.m_left_image_path,  self.m_right_image_path = dlg_filg_loader.get_image_paths()
            logger.debug(
                f"{self.m_left_image_path} \n{self.m_right_image_path}")
            # list images to db
            lf = self._list_images_with_suffix(self.m_left_image_path)
            rf = self._list_images_with_suffix(self.m_right_image_path)

            self.db.delete_data(self.DB_TABLENAME, f"WHERE 1=1")
            for litem, ritem in zip(lf, rf):
                self.db.write_data(self.DB_TABLENAME,
                                   f'null, \'{self.m_left_image_path}\', 0, \'{litem}\'')
                self.db.write_data(self.DB_TABLENAME,
                                   f'null, \'{self.m_right_image_path}\', 1, \'{ritem}\'')

            # update tree control with new images
            self.update_treectrl()

    def on_op_depth_click(self, evt):
        pass

    def on_op_rectify_click(self, evt):
        pass

    def on_op_save_click(self, evt):
        pass

    def on_tree_item_selected(self, evt):
        id = evt.GetItem()
        rootid = self.m_treectrl.GetRootItem()
        if rootid != id:
            fname = self.m_treectrl.GetItemData(id)[0]
            limage_data = cv2.imread(
                os.path.join(self.m_left_image_path, fname))
            img_h, img_w = limage_data.shape[:2]
            SCALE_RATIO = img_w / DISP_IMAGE_VIEW_W
            limage_data = cv2.cvtColor(limage_data, cv2.COLOR_BGR2RGB)
            limage_data = cv2.resize(
                limage_data, (int(img_w / SCALE_RATIO), int(img_h / SCALE_RATIO)))
            self.m_panel_image.set_cvmat(limage_data)
        else:
            self.m_panel_image.set_bitmap(
                wx.Bitmap(DISP_IMAGE_VIEW_W, DISP_IMAGE_VIEW_H))
        self.m_panel_image.Refresh()

    def create_treectrl(self):
        self.iconlist = wx.ImageList(16, 16)
        self.icon_ok = self.iconlist.Add(wx.ArtProvider.GetBitmap(
            wx.ART_INFORMATION, wx.ART_OTHER, (16, 16)))
        self.icon_q = self.iconlist.Add(wx.ArtProvider.GetBitmap(
            wx.ART_CROSS_MARK, wx.ART_OTHER, (16, 16)))
        tree = wx.TreeCtrl(self.tab)
        tree.AssignImageList(self.iconlist)
        return tree

    def update_treectrl(self, all: bool = False):
        tree = self.m_treectrl
        tree.DeleteAllItems()

        # prepare sql
        condi_str = f"WHERE cameraid=0"
        results = self.db.retrive_data(
            self.DB_TABLENAME, f'rootpath, filename', condi_str)
        fnames = [r[1] for r in results]
        # rnames = [r[0] for r in results]

        # update tree control
        dirroot = tree.AddRoot(f"Left Image Files({len(fnames)}):", image=0)
        if len(fnames) > 0:
            for fname in fnames:
                newItem = tree.AppendItem(
                    dirroot,
                    f'{fname}',
                    data=[fname])
                tree.SetItemImage(newItem, self.icon_ok)
            tree.Expand(dirroot)
            tree.SelectItem(newItem)
            tree.EnsureVisible(newItem)
            tree.EnableVisibleFocus(True)

    def _list_images_with_suffix(self, rootpath, suffix_list: list = ['png', 'jpg', 'jpeg', 'bmp']):
        images = []

        for f in os.listdir(rootpath):
            if not f.startswith('.'):
                suffix = f.rsplit('.', 1)[-1].lower()
                if suffix in suffix_list:
                    images.append(f)
        return images


class StereoFileLoader(wx.Dialog):
    def __init__(self, parent, l, r):
        wx.Dialog.__init__(
            self, parent, title="Load Stereo Images", size=wx.Size(600, 180))
        self.SetMinSize(self.GetSize())
        self.SetMaxSize(self.GetSize())

        # temp l/r path
        self.temp_lpath = l
        self.temp_rpath = r

        # init ui
        self.m_layout_main = wx.BoxSizer(wx.VERTICAL)
        self.SetSizer(self.m_layout_main)
        self.create_image_load_ui(self.m_layout_main)

        # callback
        self._register_callbacks()
        self._setup_parameter()

    def get_image_paths(self):
        return self.temp_lpath, self.temp_rpath

    def create_image_load_ui(self, parentLayout):
        # left
        m_layout_images_left = wx.BoxSizer(wx.HORIZONTAL)
        self.m_textctl_left_path = wx.TextCtrl(
            self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_textctl_left_path.Enable(False)
        self.m_btn_left = wx.Button(
            self, wx.ID_ANY, u"Select Left Image", wx.DefaultPosition, wx.DefaultSize, 0)

        m_layout_images_left.Add(
            self.m_textctl_left_path, 10, wx.EXPAND | wx.ALL, 5)
        m_layout_images_left.Add(self.m_btn_left, 1, wx.ALL, 5)

        # right
        m_layout_images_right = wx.BoxSizer(wx.HORIZONTAL)
        self.m_textctl_right_path = wx.TextCtrl(
            self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_textctl_right_path.Enable(False)
        self.m_btn_right = wx.Button(
            self, wx.ID_ANY, u"Select Right Image", wx.DefaultPosition, wx.DefaultSize, 0)

        m_layout_images_right.Add(
            self.m_textctl_right_path, 10, wx.EXPAND | wx.ALL, 5)
        m_layout_images_right.Add(self.m_btn_right, 1, wx.ALL, 5)

        # confirm
        m_layout_images_confirm = wx.StdDialogButtonSizer()
        self.m_btn_confirm_ok = wx.Button(self, wx.ID_OK)
        self.m_btn_confirm_cancel = wx.Button(self, wx.ID_CANCEL)
        m_layout_images_confirm.AddButton(self.m_btn_confirm_ok)
        m_layout_images_confirm.AddButton(self.m_btn_confirm_cancel)
        m_layout_images_confirm.Realize()

        # combine layouts
        parentLayout.Add(m_layout_images_left, 0, wx.ALL, 5)
        parentLayout.Add(m_layout_images_right, 0, wx.ALL, 5)
        parentLayout.Add(m_layout_images_confirm, 0, wx.ALL, 5)

    def on_select_file_path(self, evt):
        src_btn_id = evt.GetId()
        dlg = wx.DirDialog(None, "Select Image Folder",
                           style=wx.DD_DEFAULT_STYLE | wx.DD_NEW_DIR_BUTTON)

        if dlg.ShowModal() == wx.ID_OK:
            if src_btn_id == self.m_btn_left.GetId():
                self.temp_lpath = dlg.GetPath()
                self.m_textctl_left_path.SetValue(self.temp_lpath)
            elif src_btn_id == self.m_btn_right.GetId():
                self.temp_rpath = dlg.GetPath()
                self.m_textctl_right_path.SetValue(self.temp_rpath)
        else:
            return

        dlg.Destroy()

    def _setup_parameter(self):
        self.m_textctl_left_path.SetValue(self.temp_lpath)
        self.m_textctl_right_path.SetValue(self.temp_rpath)

    def _register_callbacks(self):
        # m_btn_left m_btn_right
        self.Bind(wx.EVT_BUTTON, self.on_select_file_path, self.m_btn_left)
        self.Bind(wx.EVT_BUTTON, self.on_select_file_path, self.m_btn_right)
