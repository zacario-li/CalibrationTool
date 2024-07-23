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

        self.m_layout_main = wx.BoxSizer(wx.VERTICAL)
        self.tab.SetSizer(self.m_layout_main)
        self.init_ui()
        self._register_all_callbacks()

    def init_db(self):
        pass

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

        self.m_btn_op_depth = wx.Button(self.tab, wx.ID_ANY, u"Compute Depth",
                                        wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_btn_op_save = wx.Button(self.tab, wx.ID_ANY, u"Save Results",
                                       wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_operations.Add(self.m_btn_op_depth, 0, wx.ALL, 1)
        m_layout_operations.Add(self.m_btn_op_save, 0, wx.ALL, 1)

        return m_layout_operations

    def _create_view_ui(self):
        m_layout_data_view = wx.BoxSizer(wx.HORIZONTAL)

        # image list
        self.m_treectrl = self.create_treectrl()
        m_layout_data_view.Add(self.m_treectrl, 1, wx.EXPAND | wx.ALL, 0)

        # disparity panel
        self.m_panel_disparity = ImagePanel(self.tab, wx.Size(640,480))
        m_layout_data_view.Add(self.m_panel_disparity, 5, wx.ALIGN_BOTTOM | wx.ALL, 5)

        # points panel
        self.m_panel_points = VTKPanel(self.tab, wx.Size(640,480))
        m_layout_data_view.Add(self.m_panel_points, 5, wx.ALIGN_BOTTOM | wx.ALL, 5)
        
        return m_layout_data_view

    def _register_all_callbacks(self):
        self.tab.Bind(wx.EVT_BUTTON, self.on_load_param_click,
                      self.m_btn_load_param)
        self.tab.Bind(wx.EVT_BUTTON, self.on_op_depth_click,
                      self.m_btn_op_depth)
        self.tab.Bind(wx.EVT_BUTTON, self.on_op_save_click, self.m_btn_op_save)
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

    def on_op_depth_click(self, evt):
        pass

    def on_op_save_click(self, evt):
        pass

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
        pass
