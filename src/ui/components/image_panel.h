#ifndef IMAGE_PANEL_H
#define IMAGE_PANEL_H

#include <wx/wx.h>
#include <wx/panel.h>
#include <opencv2/core/mat.hpp>

class ImagePanel : public wxPanel {
public:
    ImagePanel(wxWindow* parent, wxWindowID id = wxID_ANY,
               const wxPoint& pos = wxDefaultPosition,
               const wxSize& size = wxDefaultSize,
               long style = wxTAB_TRAVERSAL,
               const wxString& name = wxPanelNameStr);

    void SetBitmap(const wxBitmap& bitmap);
    void SetCvMat(const cv::Mat& mat); // Handles BGR/Gray to wxImage/wxBitmap
    void ClearPanel(); // Clears the panel to a blank state

protected:
    void OnPaint(wxPaintEvent& event);

private:
    wxBitmap current_bitmap_;
    wxSize last_size_ = wxDefaultSize; // To recreate bitmap if size changes significantly
    bool is_cleared_ = true; // Flag to indicate if panel is in cleared state
};

#endif // IMAGE_PANEL_H
