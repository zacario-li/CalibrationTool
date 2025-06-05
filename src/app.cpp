#include "app.h"
#include "ui/main_window.h" // Include the new main window header
#include <wx/image.h>     // For wxImage::AddHandler

bool CalibrationApp::OnInit() {
    if (!wxApp::OnInit()) {
        return false;
    }

    // Add wxPNGHandler to load PNG images (like the icon)
    wxImage::AddHandler(new wxPNGHandler);

    MainWindow* frame = new MainWindow(nullptr, "C++ Calibration Tool");
    frame->Show(true);
    // SetTopWindow(frame); // Not strictly necessary for a single top-level frame shown like this,
                         // but good practice if other top-level windows might be created.
                         // Show() makes it the top window implicitly in many cases.
    return true;
}
