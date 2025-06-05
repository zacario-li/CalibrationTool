#include "tab_single_cam.h"
#include <wx/dirdlg.h>
#include <wx/filedlg.h>
#include <wx/artprov.h> // For wxArtProvider
#include <wx/imaglist.h>
#include <wx/wfstream.h> // For wxFileOutputStream
#include <wx/stdpaths.h> // For wxStandardPaths
#include <nlohmann/json.hpp> // For JSON saving
#include <opencv2/imgcodecs.hpp> // For cv::imread
#include <opencv2/imgproc.hpp>   // For cv::cvtColor, cv::drawChessboardCorners
#include <iostream>     // For std::cerr
#include <algorithm>    // For std::sort, std::max_element
#include <iomanip>      // For std::fixed, std::setprecision for JSON

// Define the event type for CalibrationDoneEvent
wxDEFINE_EVENT(EVT_CALIBRATION_DONE, CalibrationDoneEvent);

using json = nlohmann::json;

namespace fs = std::filesystem;

TabSingleCamPanel::TabSingleCamPanel(wxWindow* parent)
    : wxPanel(parent, wxID_ANY) {

    LayoutControls(); // Call a separate method to set up UI

    // Bind events
    Bind(wxEVT_BUTTON, &TabSingleCamPanel::OnSelectFolder, this, ID_SELECT_FOLDER);
    Bind(wxEVT_TEXT, &TabSingleCamPanel::OnTextChanged, this, ID_ROWS_TEXT);
    Bind(wxEVT_TEXT, &TabSingleCamPanel::OnTextChanged, this, ID_COLS_TEXT);
    Bind(wxEVT_TEXT, &TabSingleCamPanel::OnTextChanged, this, ID_CELLSIZE_TEXT);
    Bind(wxEVT_BUTTON, &TabSingleCamPanel::OnCalibrate, this, ID_CALIBRATE_BTN);
    Bind(wxEVT_BUTTON, &TabSingleCamPanel::OnSaveResults, this, ID_SAVE_BTN);
    Bind(wxEVT_BUTTON, &TabSingleCamPanel::OnShowDistribution, this, ID_SHOW_DIST_BTN);
    Bind(wxEVT_TREE_SEL_CHANGED, &TabSingleCamPanel::OnTreeItemSelect, this, ID_TREE_IMAGES);
    Bind(wxEVT_TREE_ITEM_RIGHT_CLICK, &TabSingleCamPanel::OnTreeItemRightClick, this, ID_TREE_IMAGES);
    Bind(wxEVT_MENU, &TabSingleCamPanel::OnRecalibrateFromMenu, this, ID_RECALIBRATE_MENU_ITEM);

    // Bind custom event for calibration completion
    Bind(EVT_CALIBRATION_DONE_HANDLER(wxID_ANY, TabSingleCamPanel::OnCalibrationDone), this);


    UpdateButtonsState(); // Initial state
}

TabSingleCamPanel::~TabSingleCamPanel() {
    if (calibration_thread_.joinable()) {
        calibration_thread_.join(); // Ensure thread finishes before destruction
    }
    delete iconList_; // Clean up image list
}

void TabSingleCamPanel::LayoutControls() {
    wxBoxSizer* mainSizer = new wxBoxSizer(wxVERTICAL);

    // --- File Path Selection ---
    wxBoxSizer* pathHBox = new wxBoxSizer(wxHORIZONTAL);
    textCtrlFilePath_ = new wxTextCtrl(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY);
    btnSelectFolder_ = new wxButton(this, ID_SELECT_FOLDER, "Select Folder...");
    pathHBox->Add(textCtrlFilePath_, 1, wxEXPAND | wxALL, 5);
    pathHBox->Add(btnSelectFolder_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);
    mainSizer->Add(pathHBox, 0, wxEXPAND | wxALL, 5);

    // --- Checkerboard Pattern Parameters ---
    wxBoxSizer* patternHBox = new wxBoxSizer(wxHORIZONTAL);
    patternHBox->Add(new wxStaticText(this, wxID_ANY, "Board Rows (squares):"), 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);
    textCtrlRows_ = new wxTextCtrl(this, ID_ROWS_TEXT, "9"); // Default value, num of squares
    patternHBox->Add(textCtrlRows_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    patternHBox->Add(new wxStaticText(this, wxID_ANY, "Board Cols (squares):"), 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);
    textCtrlCols_ = new wxTextCtrl(this, ID_COLS_TEXT, "12"); // Default value, num of squares
    patternHBox->Add(textCtrlCols_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    patternHBox->Add(new wxStaticText(this, wxID_ANY, "Cell Size (mm):"), 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);
    textCtrlCellSize_ = new wxTextCtrl(this, ID_CELLSIZE_TEXT, "5.0");
    patternHBox->Add(textCtrlCellSize_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);
    mainSizer->Add(patternHBox, 0, wxEXPAND | wxALL, 5);

    // --- Calibration Controls ---
    wxBoxSizer* calibControlsHBox = new wxBoxSizer(wxHORIZONTAL);
    checkBoxUseCustomDetector_ = new wxCheckBox(this, wxID_ANY, "Use Custom Detector");
    calibControlsHBox->Add(checkBoxUseCustomDetector_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    btnCalibrate_ = new wxButton(this, ID_CALIBRATE_BTN, "Calibrate");
    btnCalibrate_->SetBackgroundColour(*wxGREEN);
    calibControlsHBox->Add(btnCalibrate_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    btnSaveResults_ = new wxButton(this, ID_SAVE_BTN, "Save Results");
    calibControlsHBox->Add(btnSaveResults_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    btnShowDistribution_ = new wxButton(this, ID_SHOW_DIST_BTN, "Show Distribution");
    calibControlsHBox->Add(btnShowDistribution_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);
    mainSizer->Add(calibControlsHBox, 0, wxEXPAND | wxALL, 5);

    staticTextWarning_ = new wxStaticText(this, wxID_ANY, wxEmptyString);
    mainSizer->Add(staticTextWarning_, 0, wxEXPAND | wxALL, 5);

    // --- Main View Area (Tree + Image Panel) ---
    wxBoxSizer* mainViewHBox = new wxBoxSizer(wxHORIZONTAL);

    // Tree Control
    treeCtrlImages_ = new wxTreeCtrl(this, ID_TREE_IMAGES, wxDefaultPosition, wxSize(250, -1), wxTR_DEFAULT_STYLE | wxTR_HIDE_ROOT);
    iconList_ = new wxImageList(16, 16, true, 2);
    iconList_->Add(wxArtProvider::GetBitmap(wxART_INFORMATION, wxART_OTHER, wxSize(16,16))); // OK icon
    iconList_->Add(wxArtProvider::GetBitmap(wxART_CROSS_MARK, wxART_OTHER, wxSize(16,16))); // Error/Rejected icon
    treeCtrlImages_->AssignImageList(iconList_);
    mainViewHBox->Add(treeCtrlImages_, 1, wxEXPAND | wxALL, 5);

    // Image Panel
    // Python: IMAGE_VIEW_W = 800, IMAGE_VIEW_H = 600
    imagePanelMainView_ = new ImagePanel(this, wxID_ANY, wxDefaultPosition, wxSize(800, 600));
    mainViewHBox->Add(imagePanelMainView_, 3, wxEXPAND | wxALL, 5);

    // TODO: VTK Panel if translated later
    // cameraPoseView_ = new VTKPanel(this, wxSize(200,200));
    // mainViewHBox->Add(cameraPoseView_, 1, wxEXPAND | wxALL, 5);

    mainSizer->Add(mainViewHBox, 1, wxEXPAND | wxALL, 5); // Main view area should expand

    SetSizerAndFit(mainSizer);
}


void TabSingleCamPanel::OnSelectFolder(wxCommandEvent& event) {
    wxDirDialog dirDialog(this, "Select Calibration Image Folder", "", wxDD_DEFAULT_STYLE | wxDD_DIR_MUST_EXIST);
    if (dirDialog.ShowModal() == wxID_OK) {
        currentImageDir_ = fs::path(dirDialog.GetPath().ToStdString());
        textCtrlFilePath_->SetValue(currentImageDir_.string());
        PopulateImageTree();
        ClearImagePanel();
    }
    UpdateButtonsState();
}

void TabSingleCamPanel::PopulateImageTree() {
    treeCtrlImages_->DeleteAllItems();
    image_data_store_.clear();

    if (currentImageDir_.empty() || !fs::is_directory(currentImageDir_)) {
        staticTextWarning_->SetLabel("Selected path is not a directory or is empty.");
        return;
    }

    wxTreeItemId rootId = treeCtrlImages_->AddRoot("Images"); // Hidden root

    std::vector<std::string> image_extensions = {".png", ".jpg", ".jpeg", ".bmp", ".tiff", ".tif"};
    int count = 0;
    for (const auto& entry : fs::directory_iterator(currentImageDir_)) {
        if (entry.is_regular_file()) {
            std::string ext = entry.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
            bool is_image = false;
            for (const auto& img_ext : image_extensions) {
                if (ext == img_ext) {
                    is_image = true;
                    break;
                }
            }
            if (is_image) {
                std::string filename = entry.path().filename().string();
                ImageCalibData data;
                data.file_path = entry.path();
                data.filename = filename;
                image_data_store_[filename] = data;

                // Add item to tree, initially with OK icon (index 0)
                wxTreeItemId item = treeCtrlImages_->AppendItem(rootId, filename, 0, 0, new wxTreeItemData(new wxStringClientData(filename)));
                count++;
            }
        }
    }

    if (count == 0) {
        staticTextWarning_->SetLabel("No image files found in the selected folder.");
    } else {
        staticTextWarning_->SetLabel(""); // Clear warning
        treeCtrlImages_->Expand(rootId); // Expand to show items
    }
    UpdateButtonsState();
}

void TabSingleCamPanel::OnTextChanged(wxCommandEvent& event) {
    UpdateButtonsState();
}

void TabSingleCamPanel::UpdateButtonsState() {
    bool path_ok = !textCtrlFilePath_->GetValue().IsEmpty() && fs::is_directory(currentImageDir_);
    bool params_ok = !textCtrlRows_->GetValue().IsEmpty() &&
                     !textCtrlCols_->GetValue().IsEmpty() &&
                     !textCtrlCellSize_->GetValue().IsEmpty();
    bool images_loaded = !image_data_store_.empty();

    btnCalibrate_->Enable(path_ok && params_ok && images_loaded);
    btnSaveResults_->Enable(last_calibration_result_.status == core::common::CalibErrType::CAL_OK);
    btnShowDistribution_->Enable(last_calibration_result_.status == core::common::CalibErrType::CAL_OK); // Enable if results are valid
}


void TabSingleCamPanel::OnTreeItemSelect(wxTreeEvent& event) {
    wxTreeItemId itemId = event.GetItem();
    if (itemId.IsOk() && itemId != treeCtrlImages_->GetRootItem()) {
        wxStringClientData* clientData = dynamic_cast<wxStringClientData*>(treeCtrlImages_->GetItemData(itemId));
        if (clientData) {
            std::string filename = clientData->GetData().ToStdString();
            DisplayImageAndCorners(filename);
        }
    }
}

void TabSingleCamPanel::ClearImagePanel() {
    if (imagePanelMainView_) {
        imagePanelMainView_->ClearPanel();
    }
}

void TabSingleCamPanel::DisplayImageAndCorners(const std::string& filename) {
    auto it = image_data_store_.find(filename);
    if (it == image_data_store_.end()) {
        std::cerr << "Error: Image data not found for " << filename << std::endl;
        ClearImagePanel();
        return;
    }

    ImageCalibData& data = it->second;
    cv::Mat image = cv::imread(data.file_path.string());
    if (image.empty()) {
        std::cerr << "Error: Could not load image " << data.file_path.string() << std::endl;
        ClearImagePanel();
        return;
    }

    if (data.corners_found && !data.corners.empty() && calib_board_) {
        // Draw corners if they were found and calib_board_ is initialized
        // Python used (row-1, col-1) for pattern size for drawChessboardCorners
        // CalibBoard constructor takes (squares_rows, squares_cols)
        // And findChessboardCornersSB uses (inner_cols, inner_rows) which is (squares_cols-1, squares_rows-1)
        // So, CalibBoard's (board_rows_, board_cols_) are inner corners.
        cv::drawChessboardCorners(image, cv::Size(calib_board_->get_board_rows(), calib_board_->get_board_cols()), data.corners, data.corners_found);
    }

    imagePanelMainView_->SetCvMat(image);
}


void TabSingleCamPanel::OnCalibrate(wxCommandEvent& event) {
    if (calibration_thread_.joinable()) {
        wxMessageBox("Calibration is already in progress.", "Notice", wxOK | wxICON_INFORMATION);
        return;
    }

    long board_rows_squares, board_cols_squares; // Number of SQUARES
    double cell_size_mm;

    if (!textCtrlRows_->GetValue().ToLong(&board_rows_squares) ||
        !textCtrlCols_->GetValue().ToLong(&board_cols_squares) ||
        !textCtrlCellSize_->GetValue().ToDouble(&cell_size_mm)) {
        staticTextWarning_->SetLabel("Invalid board parameters. Please enter numbers.");
        staticTextWarning_->SetForegroundColour(*wxRED);
        return;
    }
    if (board_rows_squares <=1 || board_cols_squares <=1 || cell_size_mm <=0) {
        staticTextWarning_->SetLabel("Board parameters must be positive (squares > 1).");
        staticTextWarning_->SetForegroundColour(*wxRED);
        return;
    }

    // CalibBoard expects inner corners count.
    // Python code: self.ROW_COR = row-1, self.COL_COR = col-1 (where row/col are num squares)
    // So, CalibBoard constructor needs (num_squares_rows - 1, num_squares_cols - 1) if using that convention.
    // However, the CalibBoard C++ was defined to take inner corners directly.
    // Python UI: "Number of Rows", "Number of Cols" -> these are for squares.
    // CalibBoard C++ constructor: board_rows (inner corners vert), board_cols (inner corners horiz)
    // Let's assume UI input "Board Rows" = number of squares along one dim (e.g. vertical squares)
    // "Board Cols" = number of squares along other dim (e.g. horizontal squares)
    // Then CalibBoard wants (num_horiz_inner_corners, num_vert_inner_corners)
    // num_horiz_inner_corners = board_cols_squares - 1
    // num_vert_inner_corners = board_rows_squares - 1
    // This matches Python's (ROW_COR, COL_COR) for findChessboardCornersSB.
    // So, CalibBoard(board_cols_squares - 1, board_rows_squares - 1, ...)
    int inner_corners_horz = static_cast<int>(board_cols_squares - 1); // Python's ROW_COR
    int inner_corners_vert = static_cast<int>(board_rows_squares - 1); // Python's COL_COR

    calib_board_ = std::make_unique<core::calib::CalibBoard>(
        inner_corners_horz,
        inner_corners_vert,
        static_cast<float>(cell_size_mm),
        core::calib::CalibPatternType::CHESSBOARD, // Assuming chessboard for now
        checkBoxUseCustomDetector_->IsChecked()
    );

    staticTextWarning_->SetLabel("Calibrating...");
    staticTextWarning_->SetForegroundColour(*wxBLACK);

    std::vector<fs::path> image_paths = GetImagePathsFromStore(false); // Get non-rejected image paths

    if (image_paths.empty()) {
        staticTextWarning_->SetLabel("No images available for calibration (or all are rejected).");
        staticTextWarning_->SetForegroundColour(*wxRED);
        return;
    }

    // Disable calibrate button during operation
    btnCalibrate_->Enable(false);

    progress_dialog_ = new wxProgressDialog(
            "Calibration Progress", "Starting calibration...", image_paths.size() + 2, // +2 for calibration step and finalization
            this, wxPD_APP_MODAL | wxPD_AUTO_HIDE | wxPD_CAN_ABORT);


    // Launch calibration in a separate thread
    // Pass 'this' to access members, but be careful with its lifetime if panel can be destroyed.
    // For a panel within a notebook, it usually lives as long as the notebook/frame.
    calibration_thread_ = std::thread([this, image_paths, opencv_flags = 0]() { // Pass necessary params by value/copy
        // This code runs in the worker thread
        core::calib::MonoCalibResult result = calib_board_->mono_calibrate(image_paths, true, opencv_flags);

        // Update progress dialog from worker thread (carefully, use wxQueueEvent or CallAfter for complex UI)
        // For simple updates, wxProgressDialog is often thread-safe enough for Update calls.
        if (progress_dialog_ && !progress_dialog_->Update(image_paths.size() + 1, "Calibration calculation finished.")) {
             // User aborted via progress dialog
            result.status = core::common::CalibErrType::CAL_CORNER_DET_ERR; // Treat as failure
        }


        // Send result back to main thread
        CalibrationDoneEvent event_to_send(EVT_CALIBRATION_DONE, GetId());
        event_to_send.SetEventObject(this);
        event_to_send.SetResult(result);
        wxQueueEvent(this, event_to_send.Clone()); // Use Clone() for safety with wxQueueEvent
    });
}


void TabSingleCamPanel::OnCalibrationDone(CalibrationDoneEvent& event) {
    if (progress_dialog_) {
        progress_dialog_->Update(progress_dialog_->GetRange(), "Finalizing..."); // Max progress
        // Deleting dialog here can be problematic if event comes too fast.
        // Better to defer deletion or use wxPD_AUTO_HIDE.
        // For now, let auto_hide handle it or delete it slightly later.
        // progress_dialog_->Destroy(); // Python used dlg.Destroy()
        // progress_dialog_ = nullptr;
    }
    if (progress_dialog_ != nullptr) {
        // Delay hiding or destroying to ensure messages are seen
        // A timer could be used, or simply rely on auto-hide and nullify pointer
        if (progress_dialog_->IsModal()) progress_dialog_->EndModal(wxID_OK); // If it was shown modally
        else progress_dialog_->Show(false); // Hide it
        // It's safer to schedule deletion rather than deleting immediately in an event handler
        wxWindow* dlg_to_delete = progress_dialog_;
        progress_dialog_ = nullptr; // Nullify immediately
        if (dlg_to_delete) wxTheApp->AddPendingEvent(wxCommandEvent(wxEVT_DESTROY, dlg_to_delete->GetId())); // Schedule for deletion

    }


    last_calibration_result_ = event.GetResult();

    if (last_calibration_result_.status == core::common::CalibErrType::CAL_OK) {
        staticTextWarning_->SetLabel("Calibration successful. Reprojection Error: " +
                                     wxString::Format("%.4f", last_calibration_result_.overall_reprojection_error));
        staticTextWarning_->SetForegroundColour(*wxBLUE);
    } else {
        staticTextWarning_->SetLabel("Calibration failed or was aborted. Error: " +
                                     wxString(core::common::to_string(last_calibration_result_.status)));
        staticTextWarning_->SetForegroundColour(*wxRED);
    }

    // Update image_data_store_ with results
    for(const auto& path_str : last_calibration_result_.rejected_images) {
        fs::path p(path_str);
        auto it = image_data_store_.find(p.filename().string());
        if (it != image_data_store_.end()) {
            it->second.rejected = true;
            it->second.reprojection_error = -1.0; // Mark as bad
        }
    }

    // Assuming rvecs, tvecs, and per-view errors are in the same order as successfully_calibrated_images
    // The Python code stored per-image R,T and RPJE in its DB.
    // MonoCalibResult doesn't directly have per-image RPJE yet.
    // cv::calibrateCameraExtended would give per-view errors.
    // For now, just mark successful ones as not rejected.
    // TODO: Populate per-image reprojection errors if available from MonoCalibResult.
    // For now, we'll just update the 'rejected' status based on the lists.
    // And retrieve detected corners from CalibBoard's results.

    for (size_t i = 0; i < last_calibration_result_.successfully_calibrated_images.size(); ++i) {
        fs::path p(last_calibration_result_.successfully_calibrated_images[i]);
        auto it = image_data_store_.find(p.filename().string());
        if (it != image_data_store_.end()) {
            it->second.rejected = false;
            if (i < last_calibration_result_.all_image_points.size()) { // ensure index is valid
                 it->second.corners = last_calibration_result_.all_image_points[i];
                 it->second.corners_found = true;
            }
            // it->second.reprojection_error = ... // if available
        }
    }


    UpdateTreeWithCalibrationStatus();
    UpdateButtonsState(); // Re-enable calibrate button, enable save if successful

    if (calibration_thread_.joinable()) {
        calibration_thread_.join(); // Ensure thread is joined
    }
}

void TabSingleCamPanel::UpdateTreeWithCalibrationStatus(bool highlight_max_error) {
    treeCtrlImages_->DeleteAllItems(); // Clear existing items
    wxTreeItemId rootId = treeCtrlImages_->AddRoot("Images"); // Hidden root

    if (image_data_store_.empty()) return;

    double max_err = -1.0;
    if (highlight_max_error) {
        for (const auto& pair : image_data_store_) {
            if (!pair.second.rejected && pair.second.reprojection_error > max_err) {
                max_err = pair.second.reprojection_error;
            }
        }
    }

    for (const auto& pair : image_data_store_) {
        const ImageCalibData& data = pair.second;
        wxString label = data.filename;
        int icon_idx = 0; // OK icon

        if (data.rejected) {
            label += " (Rejected)";
            icon_idx = 1; // Error/Rejected icon
        } else if (data.reprojection_error >= 0) {
            label += wxString::Format(" (Err: %.3f)", data.reprojection_error);
        } else if (data.corners_found) {
            label += " (Corners found)";
        } else {
             label += " (No corners)";
             icon_idx = 1; // Mark as problematic if no corners
        }


        wxTreeItemId item_id = treeCtrlImages_->AppendItem(rootId, label, icon_idx, icon_idx, new wxStringClientData(data.filename));

        if (highlight_max_error && !data.rejected && data.reprojection_error == max_err && max_err > 0) {
            treeCtrlImages_->SetItemTextColour(item_id, *wxRED);
        }
    }
    treeCtrlImages_->Expand(rootId);
}


std::vector<std::filesystem::path> TabSingleCamPanel::GetImagePathsFromStore(bool include_rejected) {
    std::vector<fs::path> paths;
    for(const auto& pair : image_data_store_) {
        if (include_rejected || !pair.second.rejected) {
            paths.push_back(pair.second.file_path);
        }
    }
    std::sort(paths.begin(), paths.end()); // Ensure consistent order
    return paths;
}


void TabSingleCamPanel::OnSaveResults(wxCommandEvent& event) {
    if (last_calibration_result_.status != core::common::CalibErrType::CAL_OK) {
        wxMessageBox("No valid calibration results to save.", "Error", wxOK | wxICON_ERROR);
        return;
    }

    wxFileDialog saveFileDialog(this, "Save Calibration Results", "", "camera_parameters.json",
                                "JSON files (*.json)|*.json", wxFD_SAVE | wxFD_OVERWRITE_PROMPT);

    if (saveFileDialog.ShowModal() == wxID_CANCEL) {
        return;
    }

    std::string save_path = saveFileDialog.GetPath().ToStdString();
    json out_json;

    out_json["version"] = "0.1_C++"; // Indicate C++ version
    out_json["SN"] = ""; // Serial number placeholder
    out_json["Scheme"] = "opencv";

    if (!last_calibration_result_.image_size.empty()) {
        out_json["ImageShape"] = {last_calibration_result_.image_size.width, last_calibration_result_.image_size.height};
    }

    json cam_params;
    // OpenCV distortion: k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4, tau_x, tau_y
    // Python saved: Radial (k1,k2,k3), Tangential (p1,p2)
    // last_calibration_result_.dist_coeffs is (k1,k2,p1,p2,k3) if using 5-coeff model
    // Or could be more depending on flags. Assuming 5 for now.
    if (last_calibration_result_.dist_coeffs.total() >= 5) {
        cam_params["RadialDistortion"] = {
            last_calibration_result_.dist_coeffs.at<double>(0,0), // k1
            last_calibration_result_.dist_coeffs.at<double>(0,1), // k2
            last_calibration_result_.dist_coeffs.at<double>(0,4)  // k3
        };
        cam_params["TangentialDistortion"] = {
            last_calibration_result_.dist_coeffs.at<double>(0,2), // p1
            last_calibration_result_.dist_coeffs.at<double>(0,3)  // p2
        };
    } else {
        cam_params["RadialDistortion"] = json::array();
        cam_params["TangentialDistortion"] = json::array();
    }

    cam_params["IntrinsicMatrix"] = json::array();
    if (!last_calibration_result_.camera_matrix.empty()) {
        for (int r = 0; r < last_calibration_result_.camera_matrix.rows; ++r) {
            json row = json::array();
            for (int c = 0; c < last_calibration_result_.camera_matrix.cols; ++c) {
                row.push_back(last_calibration_result_.camera_matrix.at<double>(r,c));
            }
            cam_params["IntrinsicMatrix"].push_back(row);
        }
    }

    out_json["CameraParameters"] = cam_params;
    out_json["ReprojectionError"] = last_calibration_result_.overall_reprojection_error;

    try {
        std::ofstream ofs(save_path);
        ofs << std::setw(4) << out_json << std::endl;
        ofs.close();
        wxMessageBox("Calibration results saved to " + save_path, "Success", wxOK | wxICON_INFORMATION);
    } catch (const std::exception& e) {
        wxMessageBox("Error saving file: " + std::string(e.what()), "Error", wxOK | wxICON_ERROR);
    }
}

void TabSingleCamPanel::OnShowDistribution(wxCommandEvent& event) {
    // Placeholder: In Python, this showed a DetailsImagePanel with a cv::Mat (monocheck).
    // C++: Would need to generate a similar cv::Mat (e.g., from reprojection errors)
    // and display it in a DetailsImagePanel instance.
    if (last_calibration_result_.status == core::common::CalibErrType::CAL_OK) {
        // For now, just show a message with the overall reprojection error.
        wxString msg = wxString::Format("Overall Reprojection Error: %.4f", last_calibration_result_.overall_reprojection_error);
        // TODO: Could list per-image errors if they were calculated and stored.
        wxMessageBox(msg, "Calibration Quality", wxOK | wxICON_INFORMATION);
    } else {
        wxMessageBox("No valid calibration to show distribution for.", "Info", wxOK | wxICON_INFORMATION);
    }
}


void TabSingleCamPanel::OnTreeItemRightClick(wxTreeEvent& event) {
    wxTreeItemId itemId = event.GetItem();
    if (itemId.IsOk() && itemId != treeCtrlImages_->GetRootItem()) {
        wxStringClientData* clientData = dynamic_cast<wxStringClientData*>(treeCtrlImages_->GetItemData(itemId));
        if (clientData) {
            tree_item_path_for_menu_ = clientData->GetData(); // Store filename for menu handler

            wxMenu popupMenu;
            popupMenu.Append(ID_RECALIBRATE_MENU_ITEM, "Toggle Reject & Recalibrate");
            // Could add "Delete and Recalibrate" etc.

            PopupMenu(&popupMenu, event.GetPoint());
        }
    }
}

void TabSingleCamPanel::OnRecalibrateFromMenu(wxCommandEvent& event) {
    if (tree_item_path_for_menu_.IsEmpty()) return;

    std::string filename_to_toggle = tree_item_path_for_menu_.ToStdString();
    auto it = image_data_store_.find(filename_to_toggle);
    if (it != image_data_store_.end()) {
        it->second.rejected = !it->second.rejected; // Toggle rejection status
        it->second.reprojection_error = -1; // Reset error as status changed

        // If un-rejecting, ensure corners are still there or reset them
        // For simplicity, we assume corners are still valid if previously found.
        // If rejecting, the calibration will ignore it.
        // If un-rejecting, it will be included.

        UpdateTreeWithCalibrationStatus(); // Update tree display

        // Trigger recalibration if there are any non-rejected images
        bool any_valid_images = false;
        for(const auto& pair : image_data_store_) {
            if (!pair.second.rejected) {
                any_valid_images = true;
                break;
            }
        }
        if (any_valid_images && calib_board_) { // And if calib_board_ setup is still valid
            OnCalibrate(wxCommandEvent()); // Simulate calibrate button click
        } else {
            UpdateButtonsState();
        }
    }
    tree_item_path_for_menu_.Clear(); // Clear stored path
}

int TabSingleCamPanel::GetImageIndexInStore(const std::string& filename) {
    // This function is less relevant with std::map, but kept if direct indexing logic was needed.
    // For std::map, direct access `image_data_store_[filename]` or `image_data_store_.find(filename)` is used.
    return -1; // Placeholder
}
