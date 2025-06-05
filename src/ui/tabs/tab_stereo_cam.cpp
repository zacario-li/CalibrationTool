#include "tab_stereo_cam.h"
#include <wx/dirdlg.h>
#include <wx/filedlg.h>
#include <wx/artprov.h>
#include <wx/imaglist.h>
#include <wx/wfstream.h>
#include <wx/stdpaths.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <algorithm>
#include <iomanip>

// Define the event type for StereoCalibrationDoneEvent
wxDEFINE_EVENT(EVT_STEREO_CALIBRATION_DONE, StereoCalibrationDoneEvent);

using json = nlohmann::json;
namespace fs = std::filesystem;

TabStereoCamPanel::TabStereoCamPanel(wxWindow* parent)
    : wxPanel(parent, wxID_ANY) {

    current_stereo_params_.board_rows_squares = 9; // Default, num of SQUARES
    current_stereo_params_.board_cols_squares = 12; // Default, num of SQUARES
    current_stereo_params_.cell_size_mm = 5.0;  // Default

    LayoutControls();

    // Bind events
    Bind(wxEVT_BUTTON, &TabStereoCamPanel::OnOpenFileLoader, this, ID_OPEN_FILE_LOADER_BTN);
    Bind(wxEVT_BUTTON, &TabStereoCamPanel::OnCalibrate, this, ID_CALIBRATE_BTN);
    Bind(wxEVT_BUTTON, &TabStereoCamPanel::OnSaveResults, this, ID_SAVE_BTN);
    Bind(wxEVT_BUTTON, &TabStereoCamPanel::OnShowDistribution, this, ID_SHOW_DIST_BTN);
    Bind(wxEVT_TREE_SEL_CHANGED, &TabStereoCamPanel::OnTreeItemSelect, this, ID_TREE_IMAGES);
    Bind(wxEVT_TREE_ITEM_RIGHT_CLICK, &TabStereoCamPanel::OnTreeItemRightClick, this, ID_TREE_IMAGES);
    Bind(wxEVT_MENU, &TabStereoCamPanel::OnRecalibrateFromMenu, this, ID_RECALIBRATE_MENU_ITEM);

    Bind(EVT_STEREO_CALIBRATION_DONE_HANDLER(wxID_ANY, TabStereoCamPanel::OnStereoCalibrationDone), this);

    UpdateButtonsState();
}

TabStereoCamPanel::~TabStereoCamPanel() {
    if (calibration_thread_.joinable()) {
        calibration_thread_.join();
    }
    delete iconList_;
}

void TabStereoCamPanel::LayoutControls() {
    wxBoxSizer* mainSizer = new wxBoxSizer(wxVERTICAL);

    // --- Path Load Layout ---
    wxStaticBoxSizer* pathLoadSizer = new wxStaticBoxSizer(wxHORIZONTAL, this, "Image Data & Parameters");

    wxBoxSizer* pathDisplayVBox = new wxBoxSizer(wxVERTICAL);
    textCtrlLeftPath_ = new wxTextCtrl(pathLoadSizer->GetStaticBox(), wxID_ANY, "Left images path...", wxDefaultPosition, wxDefaultSize, wxTE_READONLY);
    textCtrlRightPath_ = new wxTextCtrl(pathLoadSizer->GetStaticBox(), wxID_ANY, "Right images path...", wxDefaultPosition, wxDefaultSize, wxTE_READONLY);
    pathDisplayVBox->Add(textCtrlLeftPath_, 0, wxEXPAND | wxBOTTOM, 2);
    pathDisplayVBox->Add(textCtrlRightPath_, 0, wxEXPAND | wxTOP, 2);

    pathLoadSizer->Add(pathDisplayVBox, 1, wxEXPAND | wxALL, 5);
    btnOpenFileLoader_ = new wxButton(pathLoadSizer->GetStaticBox(), ID_OPEN_FILE_LOADER_BTN, "Load Images & Set Params...");
    pathLoadSizer->Add(btnOpenFileLoader_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);
    mainSizer->Add(pathLoadSizer, 0, wxEXPAND | wxALL, 5);

    // --- Action Buttons Layout ---
    wxStaticBoxSizer* actionsSizer = new wxStaticBoxSizer(wxHORIZONTAL, this, "Calibration Actions");
    checkBoxUseCustomDetector_ = new wxCheckBox(actionsSizer->GetStaticBox(), wxID_ANY, "Use Custom Detector");
    actionsSizer->Add(checkBoxUseCustomDetector_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    btnCalibrate_ = new wxButton(actionsSizer->GetStaticBox(), ID_CALIBRATE_BTN, "Calibrate");
    btnCalibrate_->SetBackgroundColour(*wxGREEN);
    actionsSizer->Add(btnCalibrate_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    btnSaveResults_ = new wxButton(actionsSizer->GetStaticBox(), ID_SAVE_BTN, "Save Results");
    actionsSizer->Add(btnSaveResults_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    btnShowDistribution_ = new wxButton(actionsSizer->GetStaticBox(), ID_SHOW_DIST_BTN, "Show Distribution"); // Placeholder
    actionsSizer->Add(btnShowDistribution_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    staticTextWarning_ = new wxStaticText(actionsSizer->GetStaticBox(), wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxST_ELLIPSIZE_END);
    actionsSizer->Add(staticTextWarning_, 1, wxALIGN_CENTER_VERTICAL | wxALL, 5);
    mainSizer->Add(actionsSizer, 0, wxEXPAND | wxALL, 5);

    // --- Main View Area (Tree + Image Panels) ---
    wxBoxSizer* mainViewHBox = new wxBoxSizer(wxHORIZONTAL);

    treeCtrlImagePairs_ = new wxTreeCtrl(this, ID_TREE_IMAGES, wxDefaultPosition, wxSize(300, -1), wxTR_DEFAULT_STYLE | wxTR_HIDE_ROOT);
    iconList_ = new wxImageList(16, 16, true, 2);
    iconList_->Add(wxArtProvider::GetBitmap(wxART_INFORMATION, wxART_OTHER, wxSize(16,16)));
    iconList_->Add(wxArtProvider::GetBitmap(wxART_CROSS_MARK, wxART_OTHER, wxSize(16,16)));
    treeCtrlImagePairs_->AssignImageList(iconList_);
    mainViewHBox->Add(treeCtrlImagePairs_, 1, wxEXPAND | wxALL, 5);

    // Image Panels VBox
    wxBoxSizer* imagePanelsVBox = new wxBoxSizer(wxVERTICAL);

    wxBoxSizer* leftImageHBox = new wxBoxSizer(wxHORIZONTAL);
    staticTextLeftName_ = new wxStaticText(this, wxID_ANY, "Left Image:");
    leftImageHBox->Add(staticTextLeftName_, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    imagePanelLeft_ = new ImagePanel(this, wxID_ANY, wxDefaultPosition, wxSize(480, 270)); // Python: IMAGE_VIEW_W = 480, IMAGE_VIEW_H = 270
    leftImageHBox->Add(imagePanelLeft_, 1, wxEXPAND);
    imagePanelsVBox->Add(leftImageHBox, 1, wxEXPAND | wxBOTTOM, 5);

    wxBoxSizer* rightImageHBox = new wxBoxSizer(wxHORIZONTAL);
    staticTextRightName_ = new wxStaticText(this, wxID_ANY, "Right Image:");
    rightImageHBox->Add(staticTextRightName_, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    imagePanelRight_ = new ImagePanel(this, wxID_ANY, wxDefaultPosition, wxSize(480, 270));
    rightImageHBox->Add(imagePanelRight_, 1, wxEXPAND);
    imagePanelsVBox->Add(rightImageHBox, 1, wxEXPAND | wxTOP, 5);

    mainViewHBox->Add(imagePanelsVBox, 2, wxEXPAND | wxALL, 5); // Give image panels more space ratio

    mainSizer->Add(mainViewHBox, 1, wxEXPAND | wxALL, 5);
    SetSizerAndFit(mainSizer);
}

void TabStereoCamPanel::OnOpenFileLoader(wxCommandEvent& event) {
    StereoFileLoaderDialog dialog(this, "Load Stereo Images & Parameters", current_stereo_params_);
    if (dialog.ShowModal() == wxID_OK) {
        current_stereo_params_ = dialog.GetData();
        textCtrlLeftPath_->SetValue(current_stereo_params_.left_folder_path);
        textCtrlRightPath_->SetValue(current_stereo_params_.right_folder_path);
        PopulateImageTree();
        ClearImagePanels();
    }
    UpdateButtonsState();
}

void TabStereoCamPanel::PopulateImageTree() {
    treeCtrlImagePairs_->DeleteAllItems();
    image_pair_data_store_.clear();

    if (current_stereo_params_.left_folder_path.empty() || current_stereo_params_.right_folder_path.empty()) {
        staticTextWarning_->SetLabel("Left or right image folder path is empty.");
        return;
    }

    fs::path left_dir(current_stereo_params_.left_folder_path);
    fs::path right_dir(current_stereo_params_.right_folder_path);

    if (!fs::is_directory(left_dir) || !fs::is_directory(right_dir)) {
        staticTextWarning_->SetLabel("Left or right path is not a valid directory.");
        return;
    }

    wxTreeItemId rootId = treeCtrlImagePairs_->AddRoot("ImagePairs");

    std::vector<std::string> image_extensions = {".png", ".jpg", ".jpeg", ".bmp", ".tiff", ".tif"};
    std::vector<fs::path> left_files, right_files;

    for (const auto& entry : fs::directory_iterator(left_dir)) {
        if (entry.is_regular_file()) {
            std::string ext = entry.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
            if (std::find(image_extensions.begin(), image_extensions.end(), ext) != image_extensions.end()) {
                left_files.push_back(entry.path());
            }
        }
    }
    for (const auto& entry : fs::directory_iterator(right_dir)) {
         if (entry.is_regular_file()) {
            std::string ext = entry.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
            if (std::find(image_extensions.begin(), image_extensions.end(), ext) != image_extensions.end()) {
                right_files.push_back(entry.path());
            }
        }
    }
    std::sort(left_files.begin(), left_files.end());
    std::sort(right_files.begin(), right_files.end());

    size_t num_pairs = std::min(left_files.size(), right_files.size());
    int count = 0;
    for (size_t i = 0; i < num_pairs; ++i) {
        // Assuming files are named such that sorted lists correspond to pairs.
        // Python version seems to imply this.
        std::string left_filename = left_files[i].filename().string();
        std::string right_filename = right_files[i].filename().string();

        StereoImagePairData data;
        data.left_file_path = left_files[i];
        data.right_file_path = right_files[i];
        image_pair_data_store_[left_filename] = data; // Use left filename as key

        wxString label = wxString::Format("%s | %s", left_filename, right_filename);
        treeCtrlImagePairs_->AppendItem(rootId, label, 0, 0, new wxStringClientData(left_filename));
        count++;
    }

    if (count == 0) {
        staticTextWarning_->SetLabel("No matching image pairs found in folders.");
    } else {
        staticTextWarning_->SetLabel("");
        treeCtrlImagePairs_->Expand(rootId);
    }
    UpdateButtonsState();
}

void TabStereoCamPanel::UpdateButtonsState() {
    bool paths_ok = !current_stereo_params_.left_folder_path.empty() && !current_stereo_params_.right_folder_path.empty();
    bool params_ok = current_stereo_params_.board_rows_squares > 1 &&
                     current_stereo_params_.board_cols_squares > 1 &&
                     current_stereo_params_.cell_size_mm > 0;
    bool images_loaded = !image_pair_data_store_.empty();

    btnCalibrate_->Enable(paths_ok && params_ok && images_loaded);
    btnSaveResults_->Enable(last_stereo_calibration_result_.status == core::common::CalibErrType::CAL_OK);
    btnShowDistribution_->Enable(last_stereo_calibration_result_.status == core::common::CalibErrType::CAL_OK);
}

void TabStereoCamPanel::OnTreeItemSelect(wxTreeEvent& event) {
    wxTreeItemId itemId = event.GetItem();
    if (itemId.IsOk() && itemId != treeCtrlImagePairs_->GetRootItem()) {
        wxStringClientData* clientData = dynamic_cast<wxStringClientData*>(treeCtrlImagePairs_->GetItemData(itemId));
        if (clientData) {
            std::string left_filename_key = clientData->GetData().ToStdString();
            DisplayImagePairAndCorners(left_filename_key);
        }
    }
}

void TabStereoCamPanel::ClearImagePanels() {
    if (imagePanelLeft_) imagePanelLeft_->ClearPanel();
    if (imagePanelRight_) imagePanelRight_->ClearPanel();
    staticTextLeftName_->SetLabel("Left Image:");
    staticTextRightName_->SetLabel("Right Image:");
}

void TabStereoCamPanel::DisplayImagePairAndCorners(const std::string& left_filename_key) {
    auto it = image_pair_data_store_.find(left_filename_key);
    if (it == image_pair_data_store_.end()) {
        std::cerr << "Error: Image pair data not found for " << left_filename_key << std::endl;
        ClearImagePanels();
        return;
    }
    StereoImagePairData& data = it->second;

    cv::Mat left_image = cv::imread(data.left_file_path.string());
    cv::Mat right_image = cv::imread(data.right_file_path.string());

    if (left_image.empty() || right_image.empty()) {
        std::cerr << "Error: Could not load image pair for " << left_filename_key << std::endl;
        ClearImagePanels();
        return;
    }

    staticTextLeftName_->SetLabel("Left: " + data.left_file_path.filename().string());
    staticTextRightName_->SetLabel("Right: " + data.right_file_path.filename().string());

    if (calib_board_) { // Check if calib_board is initialized
        if (data.corners_found_left && !data.corners_left.empty()) {
            cv::drawChessboardCorners(left_image, cv::Size(calib_board_->get_board_rows(), calib_board_->get_board_cols()), data.corners_left, data.corners_found_left);
        }
        if (data.corners_found_right && !data.corners_right.empty()) {
            cv::drawChessboardCorners(right_image, cv::Size(calib_board_->get_board_rows(), calib_board_->get_board_cols()), data.corners_right, data.corners_found_right);
        }
    }

    imagePanelLeft_->SetCvMat(left_image);
    imagePanelRight_->SetCvMat(right_image);
}


void TabStereoCamPanel::OnCalibrate(wxCommandEvent& event) {
    if (calibration_thread_.joinable()) {
        wxMessageBox("Calibration is already in progress.", "Notice", wxOK | wxICON_INFORMATION);
        return;
    }

    // Params from current_stereo_params_ (set by dialog)
    int inner_corners_horz = static_cast<int>(current_stereo_params_.board_cols_squares - 1);
    int inner_corners_vert = static_cast<int>(current_stereo_params_.board_rows_squares - 1);
    float cell_size_mm_float = static_cast<float>(current_stereo_params_.cell_size_mm);

    calib_board_ = std::make_unique<core::calib::CalibBoard>(
        inner_corners_horz,
        inner_corners_vert,
        cell_size_mm_float,
        core::calib::CalibPatternType::CHESSBOARD,
        checkBoxUseCustomDetector_->IsChecked()
    );

    staticTextWarning_->SetLabel("Calibrating stereo cameras...");
    staticTextWarning_->SetForegroundColour(*wxBLACK);

    auto image_paths_for_calib = GetImagePathsForCalibration(false);
    if (image_paths_for_calib.empty()) {
        staticTextWarning_->SetLabel("No image pairs available for calibration.");
        staticTextWarning_->SetForegroundColour(*wxRED);
        return;
    }

    std::vector<fs::path> left_paths, right_paths;
    for(const auto& pair : image_paths_for_calib) {
        left_paths.push_back(pair.first);
        right_paths.push_back(pair.second);
    }

    btnCalibrate_->Enable(false);
    progress_dialog_ = new wxProgressDialog(
            "Stereo Calibration Progress", "Starting calibration...", left_paths.size() * 2 + 2, // approx steps
            this, wxPD_APP_MODAL | wxPD_AUTO_HIDE | wxPD_CAN_ABORT);

    calibration_thread_ = std::thread([this, left_paths, right_paths, opencv_flags = 0]() {
        // CalibBoard::stereo_calibrate is currently a stub.
        // This will call the stub.
        core::calib::StereoCalibResult result = calib_board_->stereo_calibrate(left_paths, right_paths, true, opencv_flags);

        if (progress_dialog_ && !progress_dialog_->Update(left_paths.size() * 2 + 1, "Calibration calculation finished.")) {
            result.status = core::common::CalibErrType::CAL_CORNER_DET_ERR; // Aborted
        }

        StereoCalibrationDoneEvent event_to_send(EVT_STEREO_CALIBRATION_DONE, GetId());
        event_to_send.SetEventObject(this);
        event_to_send.SetResult(result);
        wxQueueEvent(this, event_to_send.Clone());
    });
}

void TabStereoCamPanel::OnStereoCalibrationDone(StereoCalibrationDoneEvent& event) {
    if (progress_dialog_) {
        progress_dialog_->Update(progress_dialog_->GetRange(), "Finalizing...");
        wxWindow* dlg_to_delete = progress_dialog_;
        progress_dialog_ = nullptr;
        if (dlg_to_delete) wxTheApp->AddPendingEvent(wxCommandEvent(wxEVT_DESTROY, dlg_to_delete->GetId()));
    }

    last_stereo_calibration_result_ = event.GetResult();

    if (last_stereo_calibration_result_.status == core::common::CalibErrType::CAL_OK) {
        staticTextWarning_->SetLabel("Stereo calibration successful. Reprojection Error: " +
                                     wxString::Format("%.4f", last_stereo_calibration_result_.overall_reprojection_error));
        staticTextWarning_->SetForegroundColour(*wxBLUE);
    } else {
        // Since CalibBoard::stereo_calibrate is a stub, it will return error by default.
        staticTextWarning_->SetLabel("Stereo calibration failed or is not implemented. Status: " +
                                     wxString(core::common::to_string(last_stereo_calibration_result_.status)));
        staticTextWarning_->SetForegroundColour(*wxRED);
    }

    // TODO: Update image_pair_data_store_ with per-pair results if CalibBoard::stereo_calibrate provides them.
    // For now, just mark as not rejected if overall success.
    // This part needs actual data from a working stereo_calibrate.
    if (last_stereo_calibration_result_.status == core::common::CalibErrType::CAL_OK) {
        // Example of how one might update based on successfully_calibrated_image_pairs_left:
        // for (const auto& path_str : last_stereo_calibration_result_.successfully_calibrated_image_pairs_left) {
        //     fs::path p(path_str);
        //     auto it = image_pair_data_store_.find(p.filename().string());
        //     if (it != image_pair_data_store_.end()) {
        //         it->second.rejected = false;
        //         // it->second.reprojection_error_left = ...
        //         // it->second.corners_left = ...
        //     }
        // }
    } else { // If calibration failed, mark all non-rejected as rejected for now, or handle more granularly
        // for (auto& pair_entry : image_pair_data_store_) {
        //     if (!pair_entry.second.rejected) { // If it was part of the attempt
        //         // pair_entry.second.rejected = true; // Or some other status
        //     }
        // }
    }


    UpdateTreeWithCalibrationStatus();
    UpdateButtonsState();

    if (calibration_thread_.joinable()) {
        calibration_thread_.join();
    }
}

std::vector<std::pair<std::filesystem::path, std::filesystem::path>> TabStereoCamPanel::GetImagePathsForCalibration(bool include_rejected) {
    std::vector<std::pair<fs::path, fs::path>> paths;
    for(const auto& pair_entry : image_pair_data_store_) {
        if (include_rejected || !pair_entry.second.rejected) {
            paths.push_back({pair_entry.second.left_file_path, pair_entry.second.right_file_path});
        }
    }
    // Sorting might be needed if order matters and map doesn't guarantee it (though std::map does by key)
    // For paths, ensure consistent pairing.
    std::sort(paths.begin(), paths.end(), [](const auto& a, const auto& b){
        return a.first < b.first;
    });
    return paths;
}


void TabStereoCamPanel::UpdateTreeWithCalibrationStatus(bool highlight_max_error) {
    treeCtrlImagePairs_->DeleteAllItems();
    wxTreeItemId rootId = treeCtrlImagePairs_->AddRoot("ImagePairs");

    if (image_pair_data_store_.empty()) return;

    // TODO: Max error highlighting for stereo would need per-pair errors.
    // double max_err_l = -1.0, max_err_r = -1.0;

    for (const auto& pair_entry : image_pair_data_store_) {
        const StereoImagePairData& data = pair_entry.second;
        wxString label = wxString::Format("%s | %s", data.left_file_path.filename().string(), data.right_file_path.filename().string());
        int icon_idx = 0; // OK

        if (data.rejected) {
            label += " (Rejected)";
            icon_idx = 1; // Error
        } else {
            // TODO: Add error display if available
            // if (data.reprojection_error_left >= 0 && data.reprojection_error_right >=0) {
            //    label += wxString::Format(" (L:%.2f R:%.2f)", data.reprojection_error_left, data.reprojection_error_right);
            // }
        }
        wxTreeItemId item_id = treeCtrlImagePairs_->AppendItem(rootId, label, icon_idx, icon_idx, new wxStringClientData(data.left_file_path.filename().string()));
        // TODO: Highlight max error if applicable
    }
    treeCtrlImagePairs_->Expand(rootId);
}


void TabStereoCamPanel::OnSaveResults(wxCommandEvent& event) {
    if (last_stereo_calibration_result_.status != core::common::CalibErrType::CAL_OK) {
        wxMessageBox("No valid stereo calibration results to save.", "Error", wxOK | wxICON_ERROR);
        return;
    }

    wxFileDialog saveFileDialog(this, "Save Stereo Calibration Results", "", "stereo_camera_parameters.json",
                                "JSON files (*.json)|*.json", wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
    if (saveFileDialog.ShowModal() == wxID_CANCEL) return;

    std::string save_path = saveFileDialog.GetPath().ToStdString();
    json out_json;
    const auto& res = last_stereo_calibration_result_;

    out_json["version"] = "0.1_C++_Stereo";
    out_json["SN"] = "";
    out_json["Scheme"] = "opencv";
    if (!res.image_size.empty()) {
        out_json["ImageShape"] = {res.image_size.width, res.image_size.height};
    }

    auto format_cam_params = [](const cv::Mat& K, const cv::Mat& D) {
        json params_json;
        if (D.total() >= 5) {
            params_json["RadialDistortion"] = {D.at<double>(0,0), D.at<double>(0,1), D.at<double>(0,4)};
            params_json["TangentialDistortion"] = {D.at<double>(0,2), D.at<double>(0,3)};
        }
        params_json["IntrinsicMatrix"] = json::array();
        if (!K.empty()) {
            for (int r=0; r<K.rows; ++r) {
                json row = json::array();
                for (int c=0; c<K.cols; ++c) row.push_back(K.at<double>(r,c));
                params_json["IntrinsicMatrix"].push_back(row);
            }
        }
        return params_json;
    };

    out_json["CameraParameters1"] = format_cam_params(res.camera_matrix1, res.dist_coeffs1);
    out_json["CameraParameters2"] = format_cam_params(res.camera_matrix2, res.dist_coeffs2);

    if (!res.R.empty()) {
        json R_json = json::array();
        for (int r=0; r<res.R.rows; ++r) {
            json row = json::array();
            for (int c=0; c<res.R.cols; ++c) row.push_back(res.R.at<double>(r,c));
            R_json.push_back(row);
        }
        out_json["RotationOfCamera2"] = R_json;
    }
    if (!res.T.empty()) { // T is 3x1
        out_json["TranslationOfCamera2"] = {res.T.at<double>(0,0), res.T.at<double>(1,0), res.T.at<double>(2,0)};
    }
    if (!res.E.empty()) { /* Save E */ }
    if (!res.F.empty()) { /* Save F */ }
    out_json["ReprojectionError"] = res.overall_reprojection_error;

    try {
        std::ofstream ofs(save_path);
        ofs << std::setw(4) << out_json << std::endl;
        ofs.close();
        wxMessageBox("Stereo calibration results saved to " + save_path, "Success", wxOK | wxICON_INFORMATION);
    } catch (const std::exception& e) {
        wxMessageBox("Error saving file: " + std::string(e.what()), "Error", wxOK | wxICON_ERROR);
    }
}

void TabStereoCamPanel::OnShowDistribution(wxCommandEvent& event) {
    // Placeholder, similar to TabSingleCamPanel
    if (last_stereo_calibration_result_.status == core::common::CalibErrType::CAL_OK) {
        wxString msg = wxString::Format("Overall Stereo Reprojection Error: %.4f", last_stereo_calibration_result_.overall_reprojection_error);
        wxMessageBox(msg, "Stereo Calibration Quality", wxOK | wxICON_INFORMATION);
    } else {
        wxMessageBox("No valid stereo calibration to show distribution for.", "Info", wxOK | wxICON_INFORMATION);
    }
}

void TabStereoCamPanel::OnTreeItemRightClick(wxTreeEvent& event) {
    wxTreeItemId itemId = event.GetItem();
    if (itemId.IsOk() && itemId != treeCtrlImagePairs_->GetRootItem()) {
        wxStringClientData* clientData = dynamic_cast<wxStringClientData*>(treeCtrlImagePairs_->GetItemData(itemId));
        if (clientData) {
            tree_item_key_for_menu_ = clientData->GetData(); // Store left filename (key)
            wxMenu popupMenu;
            popupMenu.Append(ID_RECALIBRATE_MENU_ITEM, "Toggle Reject & Recalibrate");
            PopupMenu(&popupMenu, event.GetPoint());
        }
    }
}

void TabStereoCamPanel::OnRecalibrateFromMenu(wxCommandEvent& event) {
    if (tree_item_key_for_menu_.IsEmpty()) return;
    std::string key = tree_item_key_for_menu_.ToStdString();
    auto it = image_pair_data_store_.find(key);
    if (it != image_pair_data_store_.end()) {
        it->second.rejected = !it->second.rejected;
        UpdateTreeWithCalibrationStatus();
        // Trigger recalibration if conditions are met
        if (calib_board_ && !GetImagePathsForCalibration(false).empty()) {
             OnCalibrate(wxCommandEvent());
        } else {
            UpdateButtonsState();
        }
    }
    tree_item_key_for_menu_.Clear();
}
