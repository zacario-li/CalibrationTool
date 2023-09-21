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

    def create_main_ui(self):
        m_layout_he_type = wx.BoxSizer( wx.HORIZONTAL )

        m_radioBox1Choices = [ u"AX=XB", u"AX=YB" ]
        self.m_radioBox1 = wx.RadioBox( self.tab, wx.ID_ANY, u"Calib Type", wx.DefaultPosition, wx.DefaultSize, m_radioBox1Choices, 1, wx.RA_SPECIFY_ROWS )
        self.m_radioBox1.SetSelection( 0 )
        m_layout_he_type.Add( self.m_radioBox1, 0, wx.ALIGN_CENTER_VERTICAL|wx.ALL, 5 )

        self.m_checkBox1 = wx.CheckBox( self.tab, wx.ID_ANY, u"R Only", wx.DefaultPosition, wx.DefaultSize, 0 )
        m_layout_he_type.Add( self.m_checkBox1, 0, wx.ALIGN_CENTER_VERTICAL|wx.ALL, 5 )


        self.m_layout_he_main.Add( m_layout_he_type, 1, wx.EXPAND, 5 )

        m_layout_he_calib_method = wx.BoxSizer( wx.HORIZONTAL )

        m_radioBox2Choices = [ u"TSAI", u"PARK", u"HORAUD", u"ANDREFF", u"DANIILIDIS" ]
        self.m_radioBox2 = wx.RadioBox( self.tab, wx.ID_ANY, u"AX=XB Calib Method", wx.DefaultPosition, wx.DefaultSize, m_radioBox2Choices, 1, wx.RA_SPECIFY_ROWS )
        self.m_radioBox2.SetSelection( 2 )
        m_layout_he_calib_method.Add( self.m_radioBox2, 0, wx.ALL, 5 )

        m_radioBox3Choices = [ u"SHAH", u"LI" ]
        self.m_radioBox3 = wx.RadioBox( self.tab, wx.ID_ANY, u"AX=YB Calib Method", wx.DefaultPosition, wx.DefaultSize, m_radioBox3Choices, 1, wx.RA_SPECIFY_ROWS )
        self.m_radioBox3.SetSelection( 0 )
        self.m_radioBox3.Enable( False )

        m_layout_he_calib_method.Add( self.m_radioBox3, 0, wx.ALL, 5 )


        self.m_layout_he_main.Add( m_layout_he_calib_method, 2, wx.EXPAND, 5 )

        m_layout__he_dataloader = wx.BoxSizer( wx.HORIZONTAL )

        m_layout_he_dataloader_path_main = wx.BoxSizer( wx.VERTICAL )

        m_layout_he_dataloader_A = wx.BoxSizer( wx.HORIZONTAL )

        self.m_textCtrl8 = wx.TextCtrl( self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
        m_layout_he_dataloader_A.Add( self.m_textCtrl8, 10, wx.ALL, 5 )

        self.m_button8 = wx.Button( self.tab, wx.ID_ANY, u"Load A", wx.DefaultPosition, wx.DefaultSize, 0 )
        m_layout_he_dataloader_A.Add( self.m_button8, 0, wx.ALL, 5 )


        m_layout_he_dataloader_path_main.Add( m_layout_he_dataloader_A, 10, wx.EXPAND, 5 )

        m_layout_he_dataloader_BY = wx.BoxSizer( wx.HORIZONTAL )

        self.m_textCtrl9 = wx.TextCtrl( self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
        m_layout_he_dataloader_BY.Add( self.m_textCtrl9, 10, wx.ALL, 5 )

        self.m_button9 = wx.Button( self.tab, wx.ID_ANY, u"Load B or Y", wx.DefaultPosition, wx.DefaultSize, 0 )
        m_layout_he_dataloader_BY.Add( self.m_button9, 0, wx.ALL, 5 )


        m_layout_he_dataloader_path_main.Add( m_layout_he_dataloader_BY, 10, wx.EXPAND, 5 )

        m_layout_he_dataloader_param = wx.BoxSizer( wx.HORIZONTAL )

        self.m_textCtrl10 = wx.TextCtrl( self.tab, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, 0 )
        m_layout_he_dataloader_param.Add( self.m_textCtrl10, 10, wx.ALL, 5 )

        self.m_button10 = wx.Button( self.tab, wx.ID_ANY, u"Load Camera Params", wx.DefaultPosition, wx.DefaultSize, 0 )
        m_layout_he_dataloader_param.Add( self.m_button10, 0, wx.ALL, 5 )


        m_layout_he_dataloader_path_main.Add( m_layout_he_dataloader_param, 10, wx.EXPAND, 5 )


        m_layout__he_dataloader.Add( m_layout_he_dataloader_path_main, 10, wx.EXPAND, 5 )

        self.m_btn_calib = wx.Button( self.tab, wx.ID_ANY, u"Calib", wx.DefaultPosition, wx.DefaultSize, 0 )
        m_layout__he_dataloader.Add( self.m_btn_calib, 0, wx.ALL|wx.EXPAND, 5 )

        self.m_btn_save = wx.Button( self.tab, wx.ID_ANY, u"Save", wx.DefaultPosition, wx.DefaultSize, 0 )
        m_layout__he_dataloader.Add( self.m_btn_save, 1, wx.ALL|wx.EXPAND, 5 )


        self.m_layout_he_main.Add( m_layout__he_dataloader, 3, wx.EXPAND, 5 )

        m_layout_he_view = wx.BoxSizer( wx.VERTICAL )

        self.m_bitmap3 = wx.StaticBitmap( self.tab, wx.ID_ANY, wx.NullBitmap, wx.DefaultPosition, wx.DefaultSize, 0 )
        m_layout_he_view.Add( self.m_bitmap3, 0, wx.ALL, 5 )

        self.m_statictext_calib_err_result = wx.StaticText( self.tab, wx.ID_ANY, u"MyLabel", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_statictext_calib_err_result.Wrap( -1 )

        m_layout_he_view.Add( self.m_statictext_calib_err_result, 0, wx.ALL, 5 )


        self.m_layout_he_main.Add( m_layout_he_view, 10, wx.EXPAND, 5 )
        pass

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
