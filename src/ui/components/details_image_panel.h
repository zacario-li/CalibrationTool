#ifndef DETAILS_IMAGE_PANEL_H
#define DETAILS_IMAGE_PANEL_H

#include <wx/wx.h>
#include <wx/frame.h>
#include "image_panel.h" // Contains ImagePanel
#include <opencv2/core/mat.hpp>

class DetailsImagePanel : public wxFrame {
public:
    DetailsImagePanel(wxWindow* parent, const wxString& title,
                      const wxPoint& pos = wxDefaultPosition,
                      const wxSize& size = wxSize(800, 600),
                      // Python: style=wx.STAY_ON_TOP|wx.FRAME_NO_TASKBAR
                      // wxFRAME_NO_TASKBAR is problematic for user interaction (closing)
                      // wxSTAY_ON_TOP can be annoying. Consider wxFRAME_TOOL_WINDOW for floating.
                      // Or just a normal frame. For now, using default frame style + STAY_ON_TOP.
                      long style = wxDEFAULT_FRAME_STYLE | wxSTAY_ON_TOP);

    void CommitCvData(const cv::Mat& cvmat);

private:
    void OnActivate(wxActivateEvent& event);
    void OnClose(wxCloseEvent& event);

    ImagePanel* image_panel_; // The panel that will display the image
    wxWindow* app_parent_window_; // To re-enable parent window on close
};

#endif // DETAILS_IMAGE_PANEL_H
