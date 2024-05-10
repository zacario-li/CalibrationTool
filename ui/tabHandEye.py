import wx
import os
import threading
import json
import cv2
import pickle
import csv
from multiprocessing import Pool
import numpy as np
from ui.components import ImagePanel
from utils.ophelper import *
from utils.storage import LocalStorage
from utils.calib import CalibChessboard, HandEye, load_camera_param, combine_RT, rot_2_quat
from utils.err import CalibErrType
from loguru import logger

HE_IMAGE_VIEW_W = 800
HE_IMAGE_VIEW_H = 600

class TabHandEye():
    def __init__(self, parent, tab):
        self.tab = tab
        # global var
        self.db = self.init_db()
        self.A_path = None
        self.B_path = None
        self.cam_param_path = None
        self.cam_mtx = None
        self.cam_dist = None
        # result
        self.r_error = None
        self.t_error = None
        self.X = None
        self.Z = None
        # data mapping
        self.he_calib_method_map = {
            # axxb
            'TSAI': cv2.CALIB_HAND_EYE_TSAI,
            'PARK': cv2.CALIB_HAND_EYE_PARK,
            'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
            'ANDREFF': cv2.CALIB_HAND_EYE_ANDREFF,
            'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
            # axzb
            'SHAH': cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH,
            'LI': cv2.CALIB_ROBOT_WORLD_HAND_EYE_LI}
        # init ui
        self.m_layout_he_main = wx.BoxSizer(wx.VERTICAL)
        self.tab.SetSizer(self.m_layout_he_main)
        self.init_ui()
        # callbacks
        self._register_all_callbacks()

    def _create_ui_he_type(self):
        m_layout_he_type = wx.BoxSizer(wx.HORIZONTAL)
        # icon
        icon_bitmap = wx.Image('elements/1.png', wx.BITMAP_TYPE_PNG).ConvertToBitmap()
        layout_icon = wx.StaticBitmap(self.tab, bitmap=icon_bitmap, size=(32,32))
        m_layout_he_type.Add(layout_icon,0,wx.ALIGN_CENTER|wx.ALL, 5)

        m_radioBoxChoices_calib_he_type = [u"Eye in Hand", u"Eye to Hand"]
        self.m_radioBox_calib_he_type = wx.RadioBox(self.tab, wx.ID_ANY, u"Eye Position Choose", wx.DefaultPosition,
                                                    wx.DefaultSize, m_radioBoxChoices_calib_he_type, 1, wx.RA_SPECIFY_ROWS)
        self.m_radioBox_calib_he_type.SetSelection(0)
        m_layout_he_type.Add(self.m_radioBox_calib_he_type, 0,
                             wx.ALIGN_CENTER_VERTICAL | wx.ALL, 5)

        m_radioBoxChoices_calib_type = [u"AX=XB", u"AX=ZB"]
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

    def _create_ui_he_calib_method(self):
        m_layout_he_calib_method = wx.BoxSizer(wx.HORIZONTAL)

        # icon
        icon_bitmap = wx.Image('elements/2.png', wx.BITMAP_TYPE_PNG).ConvertToBitmap()
        layout_icon = wx.StaticBitmap(self.tab, bitmap=icon_bitmap, size=(32,32))
        m_layout_he_calib_method.Add(layout_icon,0,wx.ALIGN_CENTER|wx.ALL, 5)

        m_radioBoxChoices_axxb_calib_method = [u"TSAI", u"PARK",
                                               u"HORAUD", u"ANDREFF", u"DANIILIDIS"]
        self.m_radioBox_axxb_calib_method = wx.RadioBox(self.tab, wx.ID_ANY, u"AX=XB Calib Method",
                                                        wx.DefaultPosition, wx.DefaultSize, m_radioBoxChoices_axxb_calib_method, 1, wx.RA_SPECIFY_ROWS)
        self.m_radioBox_axxb_calib_method.SetSelection(2)
        m_layout_he_calib_method.Add(
            self.m_radioBox_axxb_calib_method, 0, wx.ALL, 5)

        m_radioBoxChoices_axzb_calib_method = [u"SHAH", u"LI"]
        self.m_radioBox_axzb_calib_method = wx.RadioBox(self.tab, wx.ID_ANY, u"AX=ZB Calib Method",
                                                        wx.DefaultPosition, wx.DefaultSize, m_radioBoxChoices_axzb_calib_method, 1, wx.RA_SPECIFY_ROWS)
        self.m_radioBox_axzb_calib_method.SetSelection(0)
        self.m_radioBox_axzb_calib_method.Enable(False)

        m_layout_he_calib_method.Add(
            self.m_radioBox_axzb_calib_method, 0, wx.ALL, 5)
        
        return m_layout_he_calib_method

    def _create_ui_he_dataloader(self):
        m_layout_he_dataloader = wx.BoxSizer(wx.HORIZONTAL)

        m_layout_he_dataloader_path_main = wx.BoxSizer(wx.VERTICAL)

        m_layout_he_dataloader_A = wx.BoxSizer(wx.HORIZONTAL)

        self.m_textctrl_load_a_path = wx.TextCtrl(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_he_dataloader_A.Add(
            self.m_textctrl_load_a_path, 10, wx.ALL, 1)
        self.m_textctrl_load_a_path.Enable(False)

        self.m_checkbox_cb_rvecflag = wx.CheckBox(self.tab, wx.ID_ANY, label="Rotation Vector")
        m_layout_he_dataloader_A.Add(self.m_checkbox_cb_rvecflag,0,wx.ALIGN_CENTER_VERTICAL|wx.ALL, 1)

        self.m_btn_loadA = wx.Button(
            self.tab, wx.ID_ANY, u"Load A(quat csv file/rvec txt file)", wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_he_dataloader_A.Add(self.m_btn_loadA, 0, wx.ALL, 1)

        m_layout_he_dataloader_path_main.Add(
            m_layout_he_dataloader_A, 10, wx.EXPAND, 0)

        m_layout_he_dataloader_B = wx.BoxSizer(wx.HORIZONTAL)

        self.m_textctrl_load_b_path = wx.TextCtrl(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_he_dataloader_B.Add(
            self.m_textctrl_load_b_path, 10, wx.ALL, 1)
        self.m_textctrl_load_b_path.Enable(False)

        self.m_btn_loadB = wx.Button(
            self.tab, wx.ID_ANY, u"Load B(images)", wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_he_dataloader_B.Add(self.m_btn_loadB, 0, wx.ALL, 1)

        m_layout_he_dataloader_path_main.Add(
            m_layout_he_dataloader_B, 10, wx.EXPAND, 0)

        m_layout_he_dataloader_param = wx.BoxSizer(wx.HORIZONTAL)

        self.m_textctrl_cam_param_path = wx.TextCtrl(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_he_dataloader_param.Add(
            self.m_textctrl_cam_param_path, 10, wx.ALL, 1)
        self.m_textctrl_cam_param_path.Enable(False)

        self.m_checkbox_cb_transflag = wx.CheckBox(
            self.tab, wx.ID_ANY, label="Need Transpose")
        m_layout_he_dataloader_param.Add(
            self.m_checkbox_cb_transflag, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 1)
        self.m_btn_load_cam_param = wx.Button(
            self.tab, wx.ID_ANY, u"Load Camera Params", wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_he_dataloader_param.Add(
            self.m_btn_load_cam_param, 0, wx.ALL, 1)

        m_layout_he_dataloader_path_main.Add(
            m_layout_he_dataloader_param, 10, wx.EXPAND, 0)

        # icon
        icon_bitmap = wx.Image('elements/4.png', wx.BITMAP_TYPE_PNG).ConvertToBitmap()
        layout_icon = wx.StaticBitmap(self.tab, bitmap=icon_bitmap, size=(32,32))
        m_layout_he_dataloader.Add(layout_icon,0,wx.ALIGN_CENTER|wx.ALL, 5)

        m_layout_he_dataloader.Add(
            m_layout_he_dataloader_path_main, 10, wx.EXPAND, 0)

        self.m_btn_calib = wx.Button(
            self.tab, wx.ID_ANY, u"Calib", wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_he_dataloader.Add(self.m_btn_calib, 0, wx.ALL | wx.EXPAND, 1)
        self.m_btn_calib.Enable(False)

        self.m_btn_save = wx.Button(
            self.tab, wx.ID_ANY, u"Save", wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_he_dataloader.Add(self.m_btn_save, 1, wx.ALL | wx.EXPAND, 1)
        self.m_btn_save.Enable(False)
        return m_layout_he_dataloader

    def _create_ui_he_checkerboard_param(self):
        m_layout_he_cb_param = wx.BoxSizer(wx.HORIZONTAL)

        # icon
        icon_bitmap = wx.Image('elements/3.png', wx.BITMAP_TYPE_PNG).ConvertToBitmap()
        layout_icon = wx.StaticBitmap(self.tab, bitmap=icon_bitmap, size=(32,32))
        m_layout_he_cb_param.Add(layout_icon,0,wx.ALIGN_CENTER|wx.ALL, 5)

        self.m_statictext_cb_row = wx.StaticText(
            self.tab, wx.ID_ANY, u"Number of Rows", wx.DefaultPosition, wx.DefaultSize, 0
        )
        m_layout_he_cb_param.Add(
            self.m_statictext_cb_row, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 5
        )
        self.m_textctrl_cb_row = wx.TextCtrl(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0
        )
        m_layout_he_cb_param.Add(
            self.m_textctrl_cb_row, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 5
        )

        self.m_statictext_cb_col = wx.StaticText(
            self.tab, wx.ID_ANY, u"Number of Cols", wx.DefaultPosition, wx.DefaultSize, 0
        )
        m_layout_he_cb_param.Add(
            self.m_statictext_cb_col, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 5)
        self.m_textctrl_cb_col = wx.TextCtrl(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0
        )
        m_layout_he_cb_param.Add(
            self.m_textctrl_cb_col, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 5)

        self.m_statictext_cb_cellsize = wx.StaticText(
            self.tab, wx.ID_ANY, u"Checkerboard Cell Size(mm)", wx.DefaultPosition, wx.DefaultSize, 0
        )
        m_layout_he_cb_param.Add(
            self.m_statictext_cb_cellsize, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 5
        )
        self.m_textctrl_cb_cellsize = wx.TextCtrl(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0
        )
        m_layout_he_cb_param.Add(
            self.m_textctrl_cb_cellsize, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 5
        )

        # use libcbdetect
        self.m_checkbox_use_libcbdetect = wx.CheckBox(
            self.tab, wx.ID_ANY, u"Use Libcbdetect", wx.DefaultPosition, wx.DefaultSize, 0)
        m_layout_he_cb_param.Add(self.m_checkbox_use_libcbdetect, 0, wx.ALIGN_CENTER_VERTICAL, 0)

        return m_layout_he_cb_param

    def _create_ui_he_view(self):
        m_layout_he_view = wx.BoxSizer(wx.HORIZONTAL)

        # images list
        self.m_treectrl = self.new_treectrl()
        m_layout_he_view.Add(self.m_treectrl, 1, wx.EXPAND, 5)

        self.m_panel_checkview = ImagePanel(self.tab, wx.Size(HE_IMAGE_VIEW_W, HE_IMAGE_VIEW_H))
        m_layout_he_view.Add(self.m_panel_checkview, 5, wx.ALIGN_CENTER_VERTICAL| wx.ALL, 5)

        self.m_statictext_calib_err_result = wx.StaticText(
            self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0)
        self.m_statictext_calib_err_result.Wrap(-1)

        m_layout_he_view.Add(self.m_statictext_calib_err_result, 2, wx.ALIGN_TOP|wx.ALL, 5)
        return m_layout_he_view

    def _register_all_callbacks(self):
        self.tab.Bind(wx.EVT_RADIOBOX, self.on_calib_type_select,
                      self.m_radioBox_calib_type)
        self.tab.Bind(wx.EVT_BUTTON, self.on_select_file_path,
                      self.m_btn_loadA)
        self.tab.Bind(wx.EVT_BUTTON, self.on_select_file_path,
                      self.m_btn_loadB)
        self.tab.Bind(wx.EVT_BUTTON, self.on_select_file_path,
                      self.m_btn_load_cam_param)
        self.tab.Bind(wx.EVT_BUTTON, self.on_click_calibrate, self.m_btn_calib)
        self.tab.Bind(
            wx.EVT_BUTTON, self.on_save_calibration_results, self.m_btn_save)

        # tree 
        self.tab.Bind(wx.EVT_TREE_SEL_CHANGING, self.on_tree_item_select, self.m_treectrl)

        # text changed evt
        self.m_textctrl_cb_row.Bind(wx.EVT_TEXT, self.on_cb_text_changed)
        self.m_textctrl_cb_col.Bind(wx.EVT_TEXT, self.on_cb_text_changed)
        self.m_textctrl_cb_cellsize.Bind(wx.EVT_TEXT, self.on_cb_text_changed)

    def _update_btns(self):
        # check if A/B/Cam is ready, then enable the calib button
        a_p = self.m_textctrl_load_a_path.GetValue()
        b_p = self.m_textctrl_load_b_path.GetValue()
        c_p = self.m_textctrl_cam_param_path.GetValue()
        col_p = self.m_textctrl_cb_col.GetValue()
        row_p = self.m_textctrl_cb_row.GetValue()
        cell_p = self.m_textctrl_cb_cellsize.GetValue()

        if len(a_p) > 0 and len(b_p) > 0 and len(c_p) > 0 and len(col_p) > 0 and len(row_p) > 0 and len(cell_p) > 0:
            self.m_btn_calib.Enable()
        else:
            self.m_btn_calib.Enable(False)
        self.m_btn_save.Enable(False)

    def _list_images_with_suffix(self, rootpath: str, suffix_list: list = ['png', 'jpg', 'jpeg', 'bmp']):
        images = []
        for f in os.listdir(rootpath):
            # on macos, listdir will create a hidden file which name starts with '.', it can not be opened by opencv
            if not f.startswith('.'):
                suffix = f.rsplit('.', 1)[-1].lower()
                if suffix in suffix_list:
                    images.append(f)
        return images

    def _write_2_file(self, filename):
        paramJsonStr = {
            'version': '0.1',
            'SN': '',
            'Scheme': 'opencv',
            'AXXB': {
                'Matrix': self.X.tolist(),
                'rotation_err': self.r_error,
                'translation_err': self.t_error
            }
        }
        with open(f'{filename}', 'w') as f:
            json.dump(paramJsonStr, f, indent=4)

    def new_treectrl(self):
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
        if all is False:
            condi = f'WHERE isreject=0'
        else:
            condi = ''

        results = self.db.retrive_data(
            self.DB_TABLENAME, f'rootpath, filename, isreject', condi)
        filelist = [f[1] for f in results]
        rej_flag = [r[2] for r in results]

        dirroot = tree.AddRoot('filename', image=0)
        if len(filelist) > 0:
            for fname, rf in zip(filelist, rej_flag):
                newItem = tree.AppendItem(
                    dirroot, f'{fname}', data=f'{fname}')
                if rf == 1:
                    tree.SetItemTextColour(newItem, wx.RED)
                tree.SetItemImage(newItem, self.icon_ok)
            tree.Expand(dirroot)
            tree.SelectItem(newItem)
            tree.EnsureVisible(newItem)
            tree.EnableVisibleFocus(True)

    def init_ui(self):
        # 手眼标定类型选择区域
        m_layout_he_type = self._create_ui_he_type()
        self.m_layout_he_main.Add(m_layout_he_type, 1, wx.EXPAND, 0)

        # 手眼标定方法选择区域
        m_layout_he_calib_method = self._create_ui_he_calib_method()
        self.m_layout_he_main.Add(m_layout_he_calib_method, 2, wx.EXPAND, 0)

        # 标定板参数设定区域
        m_layout_he_cb_param = self._create_ui_he_checkerboard_param()
        self.m_layout_he_main.Add(m_layout_he_cb_param, 1, wx.EXPAND, 0)

        # 手眼标定数据选择区域
        self.m_staticline_dataloader_upper = wx.StaticLine(
            self.tab, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL)
        self.m_layout_he_main.Add(
            self.m_staticline_dataloader_upper, 0, wx.EXPAND | wx.ALL, 0)

        m_layout_he_dataloader = self._create_ui_he_dataloader()
        self.m_layout_he_main.Add(m_layout_he_dataloader, 2, wx.EXPAND, 0)

        self.m_staticline_dataloader_lower = wx.StaticLine(
            self.tab, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL)
        self.m_layout_he_main.Add(
            self.m_staticline_dataloader_lower, 0, wx.EXPAND | wx.ALL, 0)

        # 手眼标定结果查看区域
        m_layout_he_view = self._create_ui_he_view()
        self.m_layout_he_main.Add(m_layout_he_view, 20, wx.EXPAND, 0)

    def init_db(self):
        # db init
        # create a single camera calib table
        '''
        use quaternion and position to represent rotation and translation
        |id integer|filename text|isreject bool|qw float |qx float  |qy float  |qz float  |tx float|ty float| tz float| cors blob|
        |----------|-------------|-------------|---------|----------|----------|----------|--------|--------|---------|----------|
        |    0     |    img1.png |  False      |0.1085443|-0.2130855|-0.9618053|-0.1332042| -44.071| 272.898|-1388.602|byte array|
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
                            cors blob'''
        self.DB_FILENAME = ':memory:'
        self.DB_TABLENAME = 'handeye'
        db = LocalStorage(self.DB_FILENAME)
        ret = db.create_table(self.DB_TABLENAME, TABLE_SQL_STR)
        if ret is not True:
            logger.debug(f'create db table {self.DB_TABLENAME} failed')
            return None
        else:
            return db

    def on_tree_item_select(self, evt):
        id = evt.GetItem()
        rootid = self.m_treectrl.GetRootItem()
        if rootid != id:
            fname = self.m_treectrl.GetItemData(id)
            fullname = os.path.join(self.B_path, fname)
            img = cv2.imread(fullname)
            # draw corners
            #if self.m_btn_calib.IsEnabled():
            results = self.db.retrive_data(self.DB_TABLENAME, 'isreject, cors', f'WHERE filename=\'{fname}\' ')
            if results[0][0] == 0:
                _cors = results[0][1]
                if _cors is not None:
                    row = int(self.m_textctrl_cb_row.GetValue())
                    col = int(self.m_textctrl_cb_col.GetValue())
                    cellsize = float(self.m_textctrl_cb_cellsize.GetValue())
                    calib_instance = CalibChessboard(row, col, cellsize, use_libcbdet=self.m_checkbox_use_libcbdetect.GetValue())
                    cors = pickle.loads(_cors)
                    calib_instance.draw_corners(img, cors)
            img_w = img.shape[1]
            img_h = img.shape[0]
            SCALE_RATIO = img_w/HE_IMAGE_VIEW_W
            img_data = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img_data = cv2.resize(img_data, (int(img_w/SCALE_RATIO),int(img_h/SCALE_RATIO)))
            self.m_panel_checkview.set_cvmat(img_data)
        else:
            self.m_panel_checkview.set_bitmap(wx.Bitmap(HE_IMAGE_VIEW_W, HE_IMAGE_VIEW_H))
        self.m_panel_checkview.Refresh()

    def on_cb_text_changed(self, evt):
        self._update_btns()

    def on_select_file_path(self, evt):
        src_btn_id = evt.GetId()
        A_id = self.m_btn_loadA.GetId()
        Cam_id = self.m_btn_load_cam_param.GetId()
        wildcard_str = wx.FileSelectorDefaultWildcardStr

        if src_btn_id == A_id:
            wildcard_str = "*.txt" if self.m_checkbox_cb_rvecflag.IsChecked() else "*.csv"
        if src_btn_id == Cam_id:
            wildcard_str = "*.json"
        dlg = None
        if src_btn_id == A_id or src_btn_id == Cam_id:
            dlg = wx.FileDialog(self.tab, "Select file", wildcard=wildcard_str)
            if dlg.ShowModal() == wx.ID_OK:
                filepath = dlg.GetPath()
                if src_btn_id == A_id:
                    self.A_path = filepath
                    self.m_textctrl_load_a_path.SetLabel(self.A_path)
                if src_btn_id == Cam_id:
                    self.cam_param_path = filepath
                    # load cam params
                    self.cam_mtx, self.cam_dist = load_camera_param(filepath)
                    if isinstance(self.cam_mtx, np.ndarray) or isinstance(self.cam_dist, np.ndarray):
                        self.m_textctrl_cam_param_path.SetLabel(
                            self.cam_param_path)
                    else:
                        # load err, create a msg box to warn user
                        wx.MessageBox(
                            #"加载相机参数失败，请检查相机参数文件是否正确",
                            "Failed to load camera parameters, please check if the camera parameter file is correct",
                            "Error", wx.OK | wx.ICON_ERROR)
                        self.m_textctrl_cam_param_path.SetLabel("")
                        self.cam_param_path = None
        else:
            dlg = wx.DirDialog(self.tab, "Select calibration board image path",
                               style=wx.DD_DEFAULT_STYLE | wx.DD_NEW_DIR_BUTTON)
            if dlg.ShowModal() == wx.ID_OK:
                self.B_path = dlg.GetPath()
                self.m_textctrl_load_b_path.SetLabel(self.B_path)
                # delete old record
                self.db.delete_data(self.DB_TABLENAME, f'WHERE 1=1')
                # list all images in the dir
                images = self._list_images_with_suffix(self.B_path)
                if len(images)>0:
                    count = 0
                    for item in images:
                        count += 1
                        self.db.write_data(
                            self.DB_TABLENAME,
                            f'null, \'{self.B_path}\', \'{item}\', 0, null,  null, null, null, null, null, null, null')
                else:
                    wx.MessageBox(
                        "No image files found in the calibration board image path",
                        "Error", wx.OK | wx.ICON_ERROR)
                self.update_treectrl()

        # 设置按钮状态
        self._update_btns()

        dlg.Destroy()

    def on_click_calibrate(self, evt):
        dlg = wx.ProgressDialog(
            "Handeye calibration",
            "Calibrating ...",
            maximum=3,
            parent=self.tab,
            style=wx.PD_APP_MODAL | wx.PD_AUTO_HIDE
        )
        dlg.Update(1, "Calculating ...")
        results = self.db.retrive_data(
            self.DB_TABLENAME, f'rootpath, filename', '')
        images = [f[1] for f in results]
        thread = threading.Thread(
            target=self._run_handeye_calibration_task, args=(dlg, images))
        thread.start()

    def _run_handeye_calibration_task(self, dlg, images):
        if self.m_radioBox_calib_type.GetSelection() == 0:
            ret, r_c2g, t_c2g, r_e, t_e, results = self.do_axxb_calib(images)
            wx.CallAfter(self._handeye_calibration_task_done,
                         dlg, (ret, r_c2g, t_c2g, r_e, t_e, results))
        else:
            self.do_axzb_calib()
            wx.CallAfter(self._handeye_calibration_task_done, dlg, (0, 0, 0, 0, 0, 0))

    def _handeye_calibration_task_done(self, dlg, data):
        ret, r_c2g, t_c2g, r_e, t_e, results = data
        if self.m_radioBox_calib_type.GetSelection() == 0:
            # 保存reject标识
            iresults = self.db.retrive_data(self.DB_TABLENAME, f'rootpath, filename', '')
            images = [f[1] for f in iresults]
            rej_list = []
            cal_list = []
            for f in zip(images, results):
                if f[1][2] is True:
                    rej_list.append(f[0])
                else:
                    cal_list.append(f)
            self._set_rejected_flags(rej_list)
            
            # 保存rt & cors到非reject数据
            rvecs = [r[1][0] for r in cal_list]
            tvecs = [t[1][1] for t in cal_list]
            flist = [f[0] for f in cal_list]
            pts = [p[1][3] for p in cal_list]
            self._save_each_image_rt_cors(rvecs, tvecs, flist, pts)
            # update tree
            self.update_treectrl(True)

            if data[0] is not CalibErrType.CAL_OK:
                dlg.Destroy()
                wx.MessageBox(f"Calibration failed:{CalibErrType.to_string(data[0])}","Notice",wx.OK | wx.ICON_ERROR)
                self.m_statictext_calib_err_result.SetLabel(f"ERROR: {CalibErrType.to_string(data[0])}")
                self.m_btn_save.Enable(False)
                return 
            # axxb
            
            self.r_error = float(r_e)
            self.t_error = float(t_e)
            self.X = combine_RT(r_c2g, float(
                t_c2g[0]), float(t_c2g[1]), float(t_c2g[2]))
            result_string = f'AXXB Calibration Result:\n\n Rotation:\n {np.array2string(r_c2g)} \n\n Translation:\n {np.array2string(t_c2g)} \n\n Rotation err: {r_e} (degree) \n Translation err: {t_e} (mm)'
            self.m_statictext_calib_err_result.SetLabel(result_string)

            # axzb
        else:
            self.m_statictext_calib_err_result.SetLabel('')

        dlg.Update(3, "done")
        self.m_btn_save.Enable()

    def do_axxb_calib(self, images):
        a_p = self.m_textctrl_load_a_path.GetValue()
        b_p = self.m_textctrl_load_b_path.GetValue()
        c_p = self.m_textctrl_cam_param_path.GetValue()
        col_p = int(self.m_textctrl_cb_col.GetValue())
        row_p = int(self.m_textctrl_cb_row.GetValue())
        cell_p = float(self.m_textctrl_cb_cellsize.GetValue())
        # ax=xb
        he = HandEye()
        cb = CalibChessboard(row_p, col_p, cell_p, use_libcbdet=self.m_checkbox_use_libcbdetect.GetValue())

        # 读取传感器rt(NDI/IMU etc.)
        if self.m_checkbox_cb_rvecflag.IsChecked():
            r_g2n, t_g2n = he.generate_gripper2base_with_rvec_txt(a_p)
        else:
            r_g2n, t_g2n = he.generate_gripper2ndi_with_file(a_p, sensor_only=self.m_checkBox_rotation_only.IsChecked(
            ), randomtest=self.m_checkBox_rotation_only.IsChecked())
        # 加载相机参数
        mtx, dist = load_camera_param(
            c_p, self.m_checkbox_cb_transflag.IsChecked())
        # 计算图像外参
        # images = self._list_images_with_suffix(b_p)
        # results = self.db.retrive_data(
        #     self.DB_TABLENAME, f'rootpath, filename', '')
        # images = [f[1] for f in results]
        # check if A's size match B's size
        # TODO
        R_b2c = []
        t_b2c = []
        # test map
        if cb.USE_MT is True:
            results = cb.calculate_img_rt_parallel(b_p, images, mtx, dist)
        else:
            results = [cb.calculate_img_rt_mono((b_p, fname, mtx, dist)) for fname in images]
        # 判断results里面是否包含(None, None)
        if any(any(item is None for item in tup) for tup in results):
            return CalibErrType.CAL_DATA_SIZE_NOT_MATCH, None, None, None, None, results

        for idx in range(len(images)):
            R = results[idx][0]
            t = results[idx][1]
            R_b2c.append(R)
            t_b2c.append(t)
        # 获取当前手眼标定方法
        he_method = self.m_radioBox_axxb_calib_method.GetSelection()
        method_str = self.m_radioBox_axxb_calib_method.GetString(he_method)
        method_id = self.he_calib_method_map[method_str]
        if len(r_g2n) != len(R_b2c):
            logger.warning("g2n size not match b2c")
            return  CalibErrType.CAL_DATA_SIZE_NOT_MATCH, None, None, None, None, results

        r_c2g, t_c2g, r_e, t_e = he.calib_axxb(
            r_g2n, t_g2n, R_b2c, t_b2c, method_id)
        return CalibErrType.CAL_OK, r_c2g, t_c2g, r_e, t_e, results

    def do_axzb_calib(self):
        pass

    def on_save_calibration_results(self, evt):
        dlg = wx.FileDialog(self.tab, u"Save result", wildcard='*.json',
                            defaultFile='handeye_parameters', style=wx.FD_SAVE)
        if dlg.ShowModal() == wx.ID_OK:
            path = dlg.GetPath()
            self._write_2_file(path)
            # 打开当前保存的路径，方便用户查看
            open_folder(path)
        dlg.Destroy()

    def on_calib_type_select(self, evt):
        idx = self.m_radioBox_calib_type.GetSelection()
        if 0 == idx:
            self.m_radioBox_axxb_calib_method.Enable()
            self.m_radioBox_axzb_calib_method.Enable(False)
        else:
            self.m_radioBox_axxb_calib_method.Enable(False)
            self.m_radioBox_axzb_calib_method.Enable()

    def _set_rejected_flags(self, filelist):
        for f in filelist:
            self.db.modify_data(self.DB_TABLENAME,
                                f'SET isreject=1 WHERE filename=\'{f}\' ')

    def _save_each_image_rt_cors(self, rvecs, tvecs, filelist, pts):
        if len(rvecs) == len(filelist):
            for f, R, tv, _pts in zip(filelist, rvecs, tvecs, pts):
                #R, _ = cv2.Rodrigues(rv)
                q = rot_2_quat(R)
                cors_bytes = pickle.dumps(_pts)
                self.db.modify_data(self.DB_TABLENAME, f'''SET isreject=0, 
                                                        qw={float(q[0])},
                                                        qx={float(q[1])},
                                                        qy={float(q[2])},
                                                        qz={float(q[3])},
                                                        tx={float(tv[0])},
                                                        ty={float(tv[1])},
                                                        tz={float(tv[2])},
                                                        cors=? 
                                                        WHERE filename=\'{f}\' ''', (cors_bytes,))
        else:
            logger.debug(f"please check the file list")

    @staticmethod
    def _convert_row_based_quat_txt_to_csv(txtfilename:str, outputcsv:str, convert2mm=True):
        logger.debug(f"start convert txt into csv")
        headers = ['q0', 'qx', 'qy', 'qz', 'tx', 'ty', 'tz']
        with open(txtfilename, 'r') as infile, open(outputcsv, 'w', newline='') as outfile:
            writer = csv.writer(outfile)
            writer.writerow(headers)

            for line in infile:
                data = line.strip().split()
                data = [float(x) for x in data]

                if convert2mm is True:
                    data[4] *=1000
                    data[5] *=1000
                    data[6] *=1000

                writer.writerow(data)

        logger.debug(f"convert done")