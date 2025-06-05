#include "image_panel.h"
#include <wx/dcclient.h> // For wxPaintDC
#include <wx/image.h>    // For wxImage
#include <opencv2/imgproc.hpp> // For cv::cvtColor, cv::resize
#include <iostream>      // For std::cerr

ImagePanel::ImagePanel(wxWindow* parent, wxWindowID id,
                       const wxPoint& pos, const wxSize& size,
                       long style, const wxString& name)
    : wxPanel(parent, id, pos, size, style, name) {

    current_bitmap_ = wxBitmap(size, wxBITMAP_SCREEN_DEPTH); // Initialize with a valid bitmap of panel size
    last_size_ = size;
    is_cleared_ = true;

    // Set background to black or a neutral color
    SetBackgroundStyle(wxBG_STYLE_PAINT); // Required on some platforms to prevent flicker with custom OnPaint
    SetBackgroundColour(*wxBLACK);


    Bind(wxEVT_PAINT, &ImagePanel::OnPaint, this);
    // Optional: Bind EVT_SIZE if you want to handle panel resizing dynamically,
    // e.g., by scaling the bitmap or re-creating it.
    // Bind(wxEVT_SIZE, &ImagePanel::OnSize, this);
}

void ImagePanel::OnPaint(wxPaintEvent& event) {
    wxPaintDC dc(this);
    dc.Clear(); // Clear the background first

    if (is_cleared_ || !current_bitmap_.IsOk()) {
        // Panel is cleared or bitmap is invalid, dc.Clear() handled it.
        // Optionally draw a placeholder or leave it to background color.
    } else {
        // Ensure bitmap is not scaled beyond panel size if it's smaller,
        // or center it if it's larger and not scaled.
        // For now, simple draw at 0,0.
        dc.DrawBitmap(current_bitmap_, 0, 0);
    }
}

void ImagePanel::SetBitmap(const wxBitmap& bitmap) {
    if (bitmap.IsOk()) {
        current_bitmap_ = bitmap;
        is_cleared_ = false;
    } else {
        // std::cerr << "ImagePanel::SetBitmap: Provided bitmap is not OK." << std::endl;
        ClearPanel(); // Fallback to cleared state if bitmap is invalid
    }
    Refresh(); // Trigger OnPaint
}

void ImagePanel::SetCvMat(const cv::Mat& mat) {
    if (mat.empty()) {
        // std::cerr << "ImagePanel::SetCvMat: Input cv::Mat is empty." << std::endl;
        ClearPanel();
        return;
    }

    wxImage wx_img;
    if (mat.channels() == 3) { // Assuming BGR
        cv::Mat rgb_mat;
        cv::cvtColor(mat, rgb_mat, cv::COLOR_BGR2RGB);
        wx_img = wxImage(rgb_mat.cols, rgb_mat.rows, rgb_mat.data, true); // true for static data (no copy)
    } else if (mat.channels() == 1) { // Grayscale
        // wxImage needs RGB, so convert gray to RGB
        cv::Mat rgb_gray_mat;
        cv::cvtColor(mat, rgb_gray_mat, cv::COLOR_GRAY2RGB);
        wx_img = wxImage(rgb_gray_mat.cols, rgb_gray_mat.rows, rgb_gray_mat.data, true);
    } else {
        std::cerr << "ImagePanel::SetCvMat: Unsupported cv::Mat channel count: " << mat.channels() << std::endl;
        ClearPanel();
        return;
    }

    if (!wx_img.IsOk()) {
        std::cerr << "ImagePanel::SetCvMat: Failed to create wxImage." << std::endl;
        ClearPanel();
        return;
    }

    // Resize image to fit panel if necessary, maintaining aspect ratio (optional)
    // For now, direct conversion to bitmap of original wxImage size.
    // If the panel size is fixed and mat size varies, you might want to scale wx_img here.
    // wxSize panel_size = GetClientSize();
    // if (wx_img.GetWidth() != panel_size.GetWidth() || wx_img.GetHeight() != panel_size.GetHeight()) {
    //    // Simple scale: wx_img.Rescale(panel_size.GetWidth(), panel_size.GetHeight(), wxIMAGE_QUALITY_HIGH);
    //    // Or maintain aspect ratio:
    //    // wx_img.Rescale(panel_size.GetWidth(), panel_size.GetHeight(), wxIMAGE_QUALITY_BICUBIC);
    // }

    current_bitmap_ = wxBitmap(wx_img);
    is_cleared_ = false;
    Refresh();
}

void ImagePanel::ClearPanel() {
    // Create a blank bitmap of the current panel size or a default color
    wxSize current_size = GetClientSize();
    if (!current_size.IsFullySpecified()) { // If panel size is not yet determined
        current_size = last_size_;
        if(!current_size.IsFullySpecified()) current_size = wxSize(100,100); // Default fallback
    }

    current_bitmap_ = wxBitmap(current_size.GetWidth() > 0 ? current_size.GetWidth() : 100 ,
                               current_size.GetHeight() > 0 ? current_size.GetHeight() : 100,
                               wxBITMAP_SCREEN_DEPTH);

    // Optionally fill with a specific color
    wxMemoryDC memDC;
    memDC.SelectObject(current_bitmap_);
    memDC.SetBackground(*wxBLACK_BRUSH); // Or any color
    memDC.Clear();
    memDC.SelectObject(wxNullBitmap);

    is_cleared_ = true;
    Refresh();
}

// Optional: Handle panel resizing
// void ImagePanel::OnSize(wxSizeEvent& event) {
//     last_size_ = event.GetSize();
//     if (is_cleared_ || !current_bitmap_.IsOk()) {
//         // If cleared or bitmap was bad, recreate a blank one for the new size
//         ClearPanel();
//     } else {
//         // If there's a valid image, you might want to rescale it
//         // or just allow OnPaint to handle drawing it (e.g., centered or clipped)
//         // For now, just refresh. If bitmap is smaller than new size, it will draw at 0,0.
//         // If larger, it will be clipped.
//         Refresh();
//     }
//     event.Skip(); // Allow default processing
// }
