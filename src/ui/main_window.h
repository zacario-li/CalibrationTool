#pragma once
#include <wx/wx.h>
#include <wx/notebook.h>

// Forward declarations for tab panels (actual files will be created later)
class TabSingleCamPanel;
class TabStereoCamPanel;
class TabHandEyePanel;
class TabStereoDisparityPanel;

class MainWindow : public wxFrame {
public:
    MainWindow(wxWindow* parent, const wxString& title);

private:
    wxNotebook* notebook;
    // Pointers to tab panels (will be initialized with actual tab classes)
    TabSingleCamPanel* panelSingleCam;
    TabStereoCamPanel* panelStereoCam;
    TabHandEyePanel* panelHandEye;
    TabStereoDisparityPanel* panelStereoDisparity;
};
