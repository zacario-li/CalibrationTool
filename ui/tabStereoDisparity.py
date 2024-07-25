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
import vtk
from vtk.util import numpy_support
import pickle

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

        # sgbm parameters
        self.sgbm_matcher = None

        # default images path
        self.m_left_image_path = ''
        self.m_right_image_path = ''

        # last computed disparity, depth, rectified images
        self.disparity_image = None
        self.depth_image = None
        self.rectified_left_image = None
        self.rectified_right_image = None

        # setup layout
        self.m_layout_main = wx.BoxSizer(wx.VERTICAL)
        self.tab.SetSizer(self.m_layout_main)
        self.init_ui()
        self._register_all_callbacks()
        self._reset_op_btns(False)

    def init_db(self):
        '''
        |id integer|rootpath text|cameraid int|filename text|disparity blob|
        |----------|-------------|------------|-------------|--------------|
        |   0      |c:\data\L    |0           |img1.png     |array bytes   |
        |   1      |c:\data\R    |1           |img1.png     |array bytes   |
        '''
        TABLE_SQL_STR = '''id INTEGER PRIMARY KEY AUTOINCREMENT,
                            rootpath text,
                            cameraid int,
                            filename text,
                            disparity blob'''
        self.DB_FILENAME = ':memory:'
        # self.DB_FILENAME = 'disparity.db'
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
        # SGBM/SGM 参数UI
        m_layout_sgbm_params = self._create_sgbm_param_ui()
        self.m_layout_main.Add(m_layout_sgbm_params, 0, wx.EXPAND | wx.ALL, 0)
        # 按钮操作UI
        m_layout_operations = self._create_operations_ui()
        self.m_layout_main.Add(m_layout_operations, 0, wx.EXPAND | wx.ALL, 0)
        # 图像显示列表及disparity&points
        m_layout_data_view = self._create_view_ui()
        self.m_layout_main.Add(m_layout_data_view, 20, wx.EXPAND | wx.ALL, 0)

    def _reset_op_btns(self, enable=False):
        self.m_btn_op_disparity.Enable(enable)
        self.m_btn_op_depth.Enable(enable)
        self.m_btn_op_rectify.Enable(enable)
        self.m_btn_op_save.Enable(enable)

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

    def _create_sgbm_param_ui(self):
        m_layout_sgbm = wx.BoxSizer(wx.HORIZONTAL)
        return m_layout_sgbm

    def _create_operations_ui(self):
        m_layout_operations = wx.BoxSizer(wx.HORIZONTAL)

        self.m_btn_op_load_images = wx.Button(self.tab, wx.ID_ANY, u"Load Images",
                                              wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_btn_op_depth = wx.Button(self.tab, wx.ID_ANY, u"Compute Depth",
                                        wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_btn_op_rectify = wx.Button(self.tab, wx.ID_ANY, u"Rectify Images",
                                          wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_btn_op_disparity = wx.Button(self.tab, wx.ID_ANY, u"Compute Disparity",
                                            wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_btn_op_save = wx.Button(self.tab, wx.ID_ANY, u"Save Results",
                                       wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_operations.Add(self.m_btn_op_load_images, 0, wx.ALL, 1)
        m_layout_operations.Add(self.m_btn_op_disparity, 0, wx.ALL, 1)
        m_layout_operations.Add(self.m_btn_op_rectify, 0, wx.ALL, 1)
        m_layout_operations.Add(self.m_btn_op_depth, 0, wx.ALL, 1)
        m_layout_operations.Add(self.m_btn_op_save, 0, wx.ALL, 1)

        return m_layout_operations

    def _create_view_ui(self):
        m_layout_data_view = wx.BoxSizer(wx.HORIZONTAL)

        # image list
        self.m_treectrl = self.create_treectrl()
        m_layout_data_view.Add(self.m_treectrl, 2, wx.EXPAND | wx.ALL, 0)

        # image panel
        self.m_panel_image = ImagePanel(self.tab, wx.Size(576, 384))
        m_layout_data_view.Add(self.m_panel_image, 5,
                               wx.ALIGN_CENTER | wx.ALL, 5)

        # disparity panel
        self.m_panel_disparity = ImagePanel(self.tab, wx.Size(576, 384))
        m_layout_data_view.Add(self.m_panel_disparity, 5,
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
        self.tab.Bind(wx.EVT_BUTTON, self.on_op_disparity_click,
                      self.m_btn_op_disparity)
        self.tab.Bind(wx.EVT_BUTTON, self.on_op_save_click,
                      self.m_btn_op_save)
        self.tab.Bind(wx.EVT_TREE_SEL_CHANGING,
                      self.on_tree_item_selected,
                      self.m_treectrl)

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
            # init stereo parameters
            self.sgbminstance = SgbmCpu(filepath)
            self.sgbm_matcher = self.sgbminstance.stereo

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
                                   f'null, \'{self.m_left_image_path}\', 0, \'{litem}\', null')
                self.db.write_data(self.DB_TABLENAME,
                                   f'null, \'{self.m_right_image_path}\', 1, \'{ritem}\', null')

            # update tree control with new images
            self.update_treectrl()

    def on_op_depth_click(self, evt):
        if self.disparity_image is not None:
            logger.debug("Computing depth...")
            disp = self.disparity_image.astype(np.float32)/16.0
            # disp = cv2.convertScaleAbs(self.disparity_image, alpha=1/16.0, beta=0)
            self.depth_image = cv2.reprojectImageTo3D(
                disp, self.sgbminstance.Q, handleMissingValues=True)

            h, w, _ = self.depth_image.shape
            depths = np.linalg.norm(self.depth_image, axis=2)
            valid_mask = np.isfinite(depths)
            valid_depths = depths[valid_mask]

            min_depth = np.min(valid_depths)
            max_depth = np.max(valid_depths)
            depth_range = max_depth - min_depth

            points = vtk.vtkPoints()
            colors = vtk.vtkUnsignedCharArray()
            colors.SetNumberOfComponents(3)
            colors.SetName("Colors")

            for y in range(h):
                for x in range(w):
                    point = self.depth_image[y, x]
                    if not np.isfinite(point[2]) or point[2] < min_depth or point[2] > max_depth:
                        continue
                    points.InsertNextPoint(point[0], point[1], point[2])

            polydata = vtk.vtkPolyData()
            polydata.SetPoints(points)
            polydata.GetPointData().SetScalars(colors)

            vertex_filter = vtk.vtkVertexGlyphFilter()
            vertex_filter.SetInputData(polydata)
            vertex_filter.Update()

            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(vertex_filter.GetOutput())

            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetPointSize(2)

            renderer = vtk.vtkRenderer()
            renderWindow = vtk.vtkRenderWindow()
            renderWindow.AddRenderer(renderer)
            renderWindow.SetSize(1280, 800)

            renderer.AddActor(actor)

            interactor = vtk.vtkRenderWindowInteractor()
            interactor.SetRenderWindow(renderWindow)

            style = vtk.vtkInteractorStyleTrackballCamera()
            interactor.SetInteractorStyle(style)

            interactor.Initialize()
            renderWindow.Render()

            interactor.Start()

    def on_op_rectify_click(self, evt):
        pass

    def on_op_disparity_click(self, evt):
        item = self.m_treectrl.GetFocusedItem()
        lfname = self.m_treectrl.GetItemData(item)[0]
        rfname = self.m_treectrl.GetItemData(item)[1]
        # check if fname is exist in folder
        if os.path.isfile(os.path.join(self.m_left_image_path, lfname)) and os.path.isfile(os.path.join(self.m_right_image_path, rfname)):
            # 以下代码非常耗时，我希望统一放在线程中运行
            dlg = wx.ProgressDialog("Disparity",
                                    "Computing now，please wait",
                                    maximum=10,
                                    parent=self.tab,
                                    style=wx.PD_APP_MODAL | wx.PD_AUTO_HIDE)
            dlg.Update(5)

            thread = threading.Thread(target=self.do_compute_disparity_task, args=(lfname, rfname, dlg))
            thread.start()

    def on_op_save_click(self, evt):
        item = self.m_treectrl.GetFocusedItem()
        lfname = self.m_treectrl.GetItemData(item)[0]
        # retrive disparity from db
        condi_disp = f"WHERE cameraid=0 AND filename=\'{lfname}\' "
        disp_data = self.db.retrive_data(self.DB_TABLENAME, f'disparity', condi_disp)
        if disp_data[0][0] is not None:
            disp = pickle.loads(disp_data[0][0])
            disparity_disp = cv2.normalize(disp.astype(np.uint8), None, 0, 255, cv2.NORM_MINMAX)
            cv2.imwrite("disparity.png", disparity_disp)
            wx.MessageBox(f"Disparity saved to disparity.png!", "Info", wx.OK)


    def on_tree_item_selected(self, evt):
        id = evt.GetItem()
        rootid = self.m_treectrl.GetRootItem()
        if rootid != id:
            fname = self.m_treectrl.GetItemData(id)[0]
            limage_data = cv2.imread(
                os.path.join(self.m_left_image_path, fname))
            img_h, img_w = limage_data.shape[:2]
            SCALE_RATIO = img_w / DISP_IMAGE_VIEW_W
            # limage_data = cv2.cvtColor(limage_data, cv2.COLOR_BGR2RGB)
            limage_data = cv2.resize(
                limage_data, (int(img_w / SCALE_RATIO), int(img_h / SCALE_RATIO)))
            self.m_panel_image.set_cvmat(limage_data)

            # retrive disparity from db
            condi_disp = f"WHERE cameraid=0 AND filename=\'{fname}\' "
            disp_data = self.db.retrive_data(self.DB_TABLENAME, f'disparity', condi_disp)
            if disp_data[0][0] is not None:
                disp = pickle.loads(disp_data[0][0])
                disparity_display = cv2.normalize(disp.astype(
                    np.uint8), None, 0, 255, cv2.NORM_MINMAX)
                h,w = disparity_display.shape
                SCALE_RATIO = w / DISP_IMAGE_VIEW_W
                disparity_display = cv2.resize(
                    disparity_display, (int(w/SCALE_RATIO), int(h/SCALE_RATIO)))
                self.m_panel_disparity.set_cvmat(disparity_display)
                self._reset_op_btns(True)
            else:
                self._clear_image_panel(self.m_panel_disparity)
                self._reset_op_btns(False)
            self.m_btn_op_disparity.Enable(True)
        else:
            self._clear_image_panel(self.m_panel_image)
            self._clear_image_panel(self.m_panel_disparity)
            self._reset_op_btns(False)
            self.m_btn_op_disparity.Enable(False)
            
        self.m_panel_image.Refresh()
        self.m_panel_disparity.Refresh()

    def do_compute_disparity_task(self, lfname, rfname, dlg):
        limage = cv2.imread(os.path.join(self.m_left_image_path, lfname))
        rimage = cv2.imread(os.path.join(self.m_right_image_path, rfname))
        rl = cv2.remap(limage, self.sgbminstance.map1x, self.sgbminstance.map1y, cv2.INTER_LINEAR)
        rr = cv2.remap(rimage, self.sgbminstance.map2x, self.sgbminstance.map2y, cv2.INTER_LINEAR)
        self.disparity_image = self.sgbm_matcher.compute(rl, rr)

        wx.CallAfter(self.on_disparity_done, dlg, lfname, rfname)

    def on_disparity_done(self, dlg, lfname, rfname):
        # save disparity into db
        disparity_bytes = pickle.dumps(self.disparity_image)
        self.db.modify_data(self.DB_TABLENAME,
                            f'''SET disparity=? 
                            WHERE cameraid=0 AND filename=\'{lfname}\'
                            ''',
                            (disparity_bytes,))

        dlg.Update(10)
        disparity_display = cv2.normalize(self.disparity_image.astype(
            np.uint8), None, 0, 255, cv2.NORM_MINMAX)
        h,w = disparity_display.shape
        SCALE_RATIO = w / DISP_IMAGE_VIEW_W
        disparity_display = cv2.resize(
            disparity_display, (int(w/SCALE_RATIO), int(h/SCALE_RATIO)))
        self.m_panel_disparity.set_cvmat(disparity_display)
        self.m_panel_disparity.Refresh()
        dlg.Destroy()
        self._reset_op_btns(True)

    def _clear_image_panel(self, panel):
        panel.set_bitmap(wx.Bitmap(DISP_IMAGE_VIEW_W, DISP_IMAGE_VIEW_H))

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

        # clear image panels
        self._clear_image_panel(self.m_panel_disparity)
        self._clear_image_panel(self.m_panel_image)

        # prepare sql
        condi_l = f"WHERE cameraid=0"
        condi_r = f"WHERE cameraid=1"
        lresults = self.db.retrive_data(
            self.DB_TABLENAME, f'rootpath, filename', condi_l)
        rresults = self.db.retrive_data(
            self.DB_TABLENAME, f'rootpath, filename', condi_r)
        lfnames = [r[1] for r in lresults]
        rfnames = [r[1] for r in rresults]

        # update tree control
        dirroot = tree.AddRoot(f"Left Image Files({len(lfnames)}):", image=0)
        if len(lfnames) > 0:
            for l, r in zip(lfnames, rfnames):
                newItem = tree.AppendItem(
                    dirroot,
                    f'{l}',
                    data=[l, r])
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
