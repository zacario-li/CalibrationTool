#include "main_window.h"
#include <wx/panel.h> // For wxPanel placeholders
#include <wx/log.h>   // For wxLogError

// Placeholder includes for tab panel headers - these will be created later
// For now, we'll just use wxPanel as placeholders for tab content.
// #include "tabs/tab_single_cam.h"
// #include "tabs/tab_stereo_cam.h"
// #include "tabs/tab_hand_eye.h"
// #include "tabs/tab_stereo_disparity.h"

MainWindow::MainWindow(wxWindow* parent, const wxString& title)
    : wxFrame(parent, wxID_ANY, title, wxDefaultPosition, wxSize(1366, 900)) {

    SetMinSize(GetSize());
    // wxFrame::SetMaxSize is not a standard wxWidgets function.
    // SetMaxSize(GetSize()); // This is not standard.

    Centre(wxBOTH);

    try {
        // This assumes the execution directory allows relative access to 'resources/icons/calib_icon.png'
        // For robust deployment, consider using wxStandardPaths or resource embedding.
        SetIcon(wxIcon("resources/icons/calib_icon.png", wxBITMAP_TYPE_PNG));
    } catch (const wxRuntimeError& e) { // Catching wxRuntimeError or wxException
        wxLogError("Failed to load icon: %s. Ensure 'resources/icons/calib_icon.png' exists.", e.GetString());
    }

    notebook = new wxNotebook(this, wxID_ANY);

    // --- Placeholder Tab Instantiation ---
    // In the Python code, custom classes derived from wx.Panel are instantiated here.
    // For now, we use basic wxPanel. Later, these will be replaced with
    // instances of TabSingleCamPanel, TabStereoCamPanel, etc.

    wxPanel* pageSingle = new wxPanel(notebook); // Placeholder for TabSingleCam content
    // panelSingleCam = new TabSingleCamPanel(pageSingle); // This would be the pattern
    notebook->AddPage(pageSingle, "Mono Camera");

    wxPanel* pageStereo = new wxPanel(notebook); // Placeholder for TabStereoCam content
    // panelStereoCam = new TabStereoCamPanel(pageStereo);
    notebook->AddPage(pageStereo, "Stereo Camera");

    wxPanel* pageHandEye = new wxPanel(notebook); // Placeholder for TabHandEye content
    // panelHandEye = new TabHandEyePanel(pageHandEye);
    notebook->AddPage(pageHandEye, "HandEye");

    wxPanel* pageDisparity = new wxPanel(notebook); // Placeholder for TabStereoDisparity content
    // panelStereoDisparity = new TabStereoDisparityPanel(pageDisparity);
    notebook->AddPage(pageDisparity, "Stereo Disparity");

    // Initialize member pointers to nullptr for now, as actual tab classes are not yet integrated.
    panelSingleCam = nullptr;
    panelStereoCam = nullptr;
    panelHandEye = nullptr;
    panelStereoDisparity = nullptr;

    // The Python code doesn't explicitly add the notebook to a sizer in MainWindow,
    // but it's good practice for wxWidgets frames.
    wxBoxSizer* sizer = new wxBoxSizer(wxVERTICAL);
    sizer->Add(notebook, 1, wxEXPAND); // Make notebook expand to fill the frame
    SetSizer(sizer);
}
