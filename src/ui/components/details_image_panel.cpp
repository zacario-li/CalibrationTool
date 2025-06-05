#include "details_image_panel.h"
#include <opencv2/imgproc.hpp> // For cv::resize
#include <wx/sizer.h>
#include <iostream>

DetailsImagePanel::DetailsImagePanel(wxWindow* parent, const wxString& title,
                                       const wxPoint& pos, const wxSize& size, long style)
    : wxFrame(parent, wxID_ANY, title, pos, size, style), app_parent_window_(parent) {

    // Ensure the frame has a minimum size
    SetMinSize(size);
    // SetMaxSize(size); // Python code had this, but it makes frame non-resizable. Usually not desired for wxFrame.

    image_panel_ = new ImagePanel(this, wxID_ANY, wxDefaultPosition, GetClientSize());

    wxBoxSizer* main_sizer = new wxBoxSizer(wxVERTICAL);
    main_sizer->Add(image_panel_, 1, wxEXPAND | wxALL, 0); // Expand image_panel to fill frame
    SetSizer(main_sizer);

    Centre(wxBOTH); // Center the frame on screen or parent

    Bind(wxEVT_ACTIVATE, &DetailsImagePanel::OnActivate, this);
    Bind(wxEVT_CLOSE_WINDOW, &DetailsImagePanel::OnClose, this);

    // In Python, parent was disabled: self.tab.GetParent().GetParent().Disable()
    // This is risky if 'parent' is not the main app frame or if this dialog is modal.
    // For a non-modal frame like this, explicitly disabling parent might be okay,
    // but ensure it's re-enabled correctly.
    if (app_parent_window_) {
        // Check if parent is the top-level frame, to avoid disabling intermediate dialogs/windows
        wxWindow* top_level_parent = wxGetTopLevelParent(app_parent_window_);
        if (top_level_parent) {
            top_level_parent->Disable();
        } else {
            app_parent_window_->Disable();
        }
    }
}

void DetailsImagePanel::CommitCvData(const cv::Mat& cvmat) {
    if (cvmat.empty()) {
        std::cerr << "DetailsImagePanel::CommitCvData: Input cv::Mat is empty." << std::endl;
        image_panel_->ClearPanel();
        return;
    }

    cv::Mat resized_cvmat;
    // GetClientSize might not be accurate immediately after creation or if sizers are complex.
    // It's better to let ImagePanel handle its own size or pass the target size explicitly.
    // For now, using GetClientSize of the frame, which ImagePanel will use.
    wxSize target_size = GetClientSize();
    if (target_size.GetWidth() <=0 || target_size.GetHeight() <=0) {
        target_size = GetSize(); // Fallback to frame's full size if client size is zero
    }


    if (cvmat.cols != target_size.GetWidth() || cvmat.rows != target_size.GetHeight()) {
        if (target_size.GetWidth() > 0 && target_size.GetHeight() > 0) {
             cv::resize(cvmat, resized_cvmat, cv::Size(target_size.GetWidth(), target_size.GetHeight()));
        } else {
            resized_cvmat = cvmat.clone(); // No valid target size, use original
            std::cerr << "DetailsImagePanel::CommitCvData: Target panel size is zero, not resizing." << std::endl;
        }
    } else {
        resized_cvmat = cvmat.clone(); // No resize needed, but clone to be safe if cvmat is temporary
    }

    image_panel_->SetCvMat(resized_cvmat);
}

void DetailsImagePanel::OnActivate(wxActivateEvent& event) {
    // Python: self.SetFocus()
    // This is usually handled by wxWidgets automatically for frames.
    // Explicitly setting focus can sometimes be useful if there are specific child controls.
    if (event.GetActive()) {
        // Optionally set focus to a specific child, e.g., image_panel_
        // image_panel_->SetFocus();
    }
    event.Skip();
}

void DetailsImagePanel::OnClose(wxCloseEvent& event) {
    // Python: self.GetParent().Enable(); self.Destroy()
    if (app_parent_window_) {
        wxWindow* top_level_parent = wxGetTopLevelParent(app_parent_window_);
        if (top_level_parent) {
            top_level_parent->Enable();
            // If the original parent was a dialog, it might need focus too.
            // top_level_parent->SetFocus();
        } else {
             app_parent_window_->Enable();
            // app_parent_window_->SetFocus();
        }
    }
    Destroy(); // Destroys the frame
}
