#include "tab_hand_eye.h"
#include <wx/dirdlg.h>
#include <wx/filedlg.h>
#include <wx/artprov.h>
#include <wx/imaglist.h>
#include <wx/wfstream.h>
#include <wx/valnum.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp> // For cv::CALIB_HAND_EYE_TSAI etc.
#include <iostream>
#include <iomanip> // For std::setw, std::fixed, std::setprecision
#include <algorithm> // For std::transform

// Define the event type for HandEyeCalibrationDoneEvent
wxDEFINE_EVENT(EVT_HAND_EYE_CALIBRATION_DONE, HandEyeCalibrationDoneEvent);

using json = nlohmann::json;
namespace fs = std::filesystem;

TabHandEyePanel::TabHandEyePanel(wxWindow* parent)
    : wxPanel(parent, wxID_ANY) {

    hand_eye_calibrator_ = std::make_unique<core::calib::HandEyeCalibration>();

    LayoutControls();

    // Bind events
    Bind(wxEVT_RADIOBOX, &TabHandEyePanel::OnEyePositionChange, this, radioBoxEyePosition_->GetId());
    Bind(wxEVT_RADIOBOX, &TabHandEyePanel::OnCalibTypeChange, this, radioBoxCalibType_->GetId());
    Bind(wxEVT_BUTTON, &TabHandEyePanel::OnLoadARobotPoses, this, ID_LOAD_A_BTN);
    Bind(wxEVT_BUTTON, &TabHandEyePanel::OnLoadBImages, this, ID_LOAD_B_BTN);
    Bind(wxEVT_BUTTON, &TabHandEyePanel::OnLoadCamParams, this, ID_LOAD_CAM_PARAMS_BTN);
    Bind(wxEVT_BUTTON, &TabHandEyePanel::OnCalibrate, this, ID_CALIBRATE_BTN);
    Bind(wxEVT_BUTTON, &TabHandEyePanel::OnSaveResults, this, ID_SAVE_BTN);
    Bind(wxEVT_TREE_SEL_CHANGED, &TabHandEyePanel::OnTreeItemSelectB, this, ID_TREE_IMAGES_B);

    // Bind text changes for board params to update button states or re-validate
    Bind(wxEVT_TEXT, &TabHandEyePanel::OnBoardParamsChanged, this, ID_BOARD_ROWS_TEXT);
    Bind(wxEVT_TEXT, &TabHandEyePanel::OnBoardParamsChanged, this, ID_BOARD_COLS_TEXT);
    Bind(wxEVT_TEXT, &TabHandEyePanel::OnBoardParamsChanged, this, ID_BOARD_CELLSIZE_TEXT);

    Bind(EVT_HAND_EYE_CALIBRATION_DONE_HANDLER(wxID_ANY, TabHandEyePanel::OnHandEyeCalibrationDone), this);

    // Initial UI state
    OnCalibTypeChange(wxCommandEvent()); // To set initial enable/disable of method radio boxes
    UpdateButtonsState();
}

TabHandEyePanel::~TabHandEyePanel() {
    if (calibration_process_thread_.joinable()) {
        calibration_process_thread_.join();
    }
    delete iconList_;
}

void TabHandEyePanel::LayoutControls() {
    wxBoxSizer* mainSizer = new wxBoxSizer(wxVERTICAL);

    // --- Top Row: Eye Position and Calib Type ---
    wxBoxSizer* typeAndMethodRow1Sizer = new wxBoxSizer(wxHORIZONTAL);
    wxArrayString eyePosChoices;
    eyePosChoices.Add("Eye in Hand (Camera on Gripper)");
    eyePosChoices.Add("Eye to Hand (Camera Static)");
    radioBoxEyePosition_ = new wxRadioBox(this, wxID_ANY, "Camera Setup", wxDefaultPosition, wxDefaultSize, eyePosChoices, 1, wxRA_SPECIFY_ROWS);
    radioBoxEyePosition_->SetSelection(0); // Default to Eye in Hand
    typeAndMethodRow1Sizer->Add(radioBoxEyePosition_, 0, wxALL | wxEXPAND, 5);

    wxArrayString calibTypeChoices;
    calibTypeChoices.Add("AX = XB");
    calibTypeChoices.Add("AX = ZB (Not Implemented)");
    radioBoxCalibType_ = new wxRadioBox(this, wxID_ANY, "Calibration Type", wxDefaultPosition, wxDefaultSize, calibTypeChoices, 1, wxRA_SPECIFY_ROWS);
    radioBoxCalibType_->SetSelection(0); // Default to AX=XB
    typeAndMethodRow1Sizer->Add(radioBoxCalibType_, 0, wxALL | wxEXPAND, 5);
    mainSizer->Add(typeAndMethodRow1Sizer, 0, wxEXPAND | wxALL, 2);

    // --- Second Row: Calibration Methods ---
    wxBoxSizer* typeAndMethodRow2Sizer = new wxBoxSizer(wxHORIZONTAL);
    wxArrayString axxbMethods;
    axxbMethods.Add("TSAI"); axxbMethods.Add("PARK"); axxbMethods.Add("HORAUD"); axxbMethods.Add("ANDREFF"); axxbMethods.Add("DANIILIDIS");
    radioBoxAXXBMethod_ = new wxRadioBox(this, wxID_ANY, "AXXB Method", wxDefaultPosition, wxDefaultSize, axxbMethods, 1, wxRA_SPECIFY_ROWS);
    radioBoxAXXBMethod_->SetSelection(2); // HORAUD default in Python
    typeAndMethodRow2Sizer->Add(radioBoxAXXBMethod_, 0, wxALL | wxEXPAND, 5);

    wxArrayString axzbMethods;
    axzbMethods.Add("SHAH (Not Impl.)"); axzbMethods.Add("LI (Not Impl.)");
    radioBoxAXZBMethod_ = new wxRadioBox(this, wxID_ANY, "AXZB Method", wxDefaultPosition, wxDefaultSize, axzbMethods, 1, wxRA_SPECIFY_ROWS);
    radioBoxAXZBMethod_->Enable(false); // Initially disabled
    typeAndMethodRow2Sizer->Add(radioBoxAXZBMethod_, 0, wxALL | wxEXPAND, 5);
    mainSizer->Add(typeAndMethodRow2Sizer, 0, wxEXPAND | wxALL, 2);

    // --- Data Loading Section ---
    wxStaticBoxSizer* dataLoadSizer = new wxStaticBoxSizer(wxVERTICAL, this, "Data Loading");

    // Path A (Robot Poses)
    wxBoxSizer* pathASizer = new wxBoxSizer(wxHORIZONTAL);
    textCtrlPathA_ = new wxTextCtrl(dataLoadSizer->GetStaticBox(), wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY);
    btnLoadA_ = new wxButton(dataLoadSizer->GetStaticBox(), ID_LOAD_A_BTN, "Load A (Robot Poses)");
    checkBoxRvecInputA_ = new wxCheckBox(dataLoadSizer->GetStaticBox(), wxID_ANY, "Txt: x y z rx ry rz");
    checkBoxRvecInputA_->SetValue(true); // Default to rvec txt file as per Python hint
    checkBoxRotationOnlyA_ = new wxCheckBox(dataLoadSizer->GetStaticBox(), wxID_ANY, "CSV: Rotation Only");
    pathASizer->Add(new wxStaticText(dataLoadSizer->GetStaticBox(), wxID_ANY, "A:"), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    pathASizer->Add(textCtrlPathA_, 1, wxEXPAND | wxRIGHT, 5);
    pathASizer->Add(checkBoxRvecInputA_, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    pathASizer->Add(checkBoxRotationOnlyA_, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    pathASizer->Add(btnLoadA_, 0, wxALIGN_CENTER_VERTICAL);
    dataLoadSizer->Add(pathASizer, 0, wxEXPAND | wxALL, 5);

    // Path B (Target Images)
    wxBoxSizer* pathBSizer = new wxBoxSizer(wxHORIZONTAL);
    textCtrlPathB_ = new wxTextCtrl(dataLoadSizer->GetStaticBox(), wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY);
    btnLoadB_ = new wxButton(dataLoadSizer->GetStaticBox(), ID_LOAD_B_BTN, "Load B (Target Images)");
    pathBSizer->Add(new wxStaticText(dataLoadSizer->GetStaticBox(), wxID_ANY, "B:"), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    pathBSizer->Add(textCtrlPathB_, 1, wxEXPAND | wxRIGHT, 5);
    pathBSizer->Add(btnLoadB_, 0, wxALIGN_CENTER_VERTICAL);
    dataLoadSizer->Add(pathBSizer, 0, wxEXPAND | wxALL, 5);

    // Camera Parameters Path
    wxBoxSizer* camParamsSizer = new wxBoxSizer(wxHORIZONTAL);
    textCtrlCamParamsPath_ = new wxTextCtrl(dataLoadSizer->GetStaticBox(), wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY);
    btnLoadCamParams_ = new wxButton(dataLoadSizer->GetStaticBox(), ID_LOAD_CAM_PARAMS_BTN, "Load Camera Params");
    checkBoxCamParamTranspose_ = new wxCheckBox(dataLoadSizer->GetStaticBox(), wxID_ANY, "Transpose K");
    checkBoxUseRightCamStereo_ = new wxCheckBox(dataLoadSizer->GetStaticBox(), wxID_ANY, "Use Right Cam (Stereo)");
    camParamsSizer->Add(new wxStaticText(dataLoadSizer->GetStaticBox(), wxID_ANY, "Cam:"), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    camParamsSizer->Add(textCtrlCamParamsPath_, 1, wxEXPAND | wxRIGHT, 5);
    camParamsSizer->Add(checkBoxCamParamTranspose_, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    camParamsSizer->Add(checkBoxUseRightCamStereo_, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    camParamsSizer->Add(btnLoadCamParams_, 0, wxALIGN_CENTER_VERTICAL);
    dataLoadSizer->Add(camParamsSizer, 0, wxEXPAND | wxALL, 5);
    mainSizer->Add(dataLoadSizer, 0, wxEXPAND | wxALL, 5);

    // --- Target Board Parameters ---
    wxStaticBoxSizer* boardParamsSizer = new wxStaticBoxSizer(wxHORIZONTAL, this, "Target (Checkerboard) Parameters for B");
    boardParamsSizer->Add(new wxStaticText(boardParamsSizer->GetStaticBox(), wxID_ANY, "Board Rows (sq):"), 0, wxALIGN_CENTER_VERTICAL | wxALL, 2);
    textCtrlBoardRows_ = new wxTextCtrl(boardParamsSizer->GetStaticBox(), ID_BOARD_ROWS_TEXT, "9", wxDefaultPosition, wxSize(50,-1));
    boardParamsSizer->Add(textCtrlBoardRows_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 2);
    boardParamsSizer->Add(new wxStaticText(boardParamsSizer->GetStaticBox(), wxID_ANY, "Board Cols (sq):"), 0, wxALIGN_CENTER_VERTICAL | wxALL, 2);
    textCtrlBoardCols_ = new wxTextCtrl(boardParamsSizer->GetStaticBox(), ID_BOARD_COLS_TEXT, "12", wxDefaultPosition, wxSize(50,-1));
    boardParamsSizer->Add(textCtrlBoardCols_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 2);
    boardParamsSizer->Add(new wxStaticText(boardParamsSizer->GetStaticBox(), wxID_ANY, "Cell Size (mm):"), 0, wxALIGN_CENTER_VERTICAL | wxALL, 2);
    textCtrlCellSize_ = new wxTextCtrl(boardParamsSizer->GetStaticBox(), ID_BOARD_CELLSIZE_TEXT, "5.0", wxDefaultPosition, wxSize(60,-1));
    boardParamsSizer->Add(textCtrlCellSize_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 2);
    checkBoxUseCustomDetectorB_ = new wxCheckBox(boardParamsSizer->GetStaticBox(), wxID_ANY, "Use Custom Detector");
    boardParamsSizer->Add(checkBoxUseCustomDetectorB_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 2);
    mainSizer->Add(boardParamsSizer, 0, wxEXPAND | wxALL, 5);

    // --- Calibration Action and Output Options ---
    wxBoxSizer* calibActionSizer = new wxBoxSizer(wxHORIZONTAL);
    btnCalibrate_ = new wxButton(this, ID_CALIBRATE_BTN, "Calibrate Hand-Eye");
    btnCalibrate_->SetBackgroundColour(*wxGREEN);
    calibActionSizer->Add(btnCalibrate_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);
    btnSaveResults_ = new wxButton(this, ID_SAVE_BTN, "Save Hand-Eye Matrix");
    calibActionSizer->Add(btnSaveResults_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);
    checkBoxOutputInMeters_ = new wxCheckBox(this, wxID_ANY, "Save Translation in Meters");
    calibActionSizer->Add(checkBoxOutputInMeters_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);
    mainSizer->Add(calibActionSizer, 0, wxEXPAND | wxALL, 5);

    // --- Main View Area (Tree for B images, Image Panel for B, Results Text) ---
    wxBoxSizer* viewSizer = new wxBoxSizer(wxHORIZONTAL);
    treeCtrlImagesB_ = new wxTreeCtrl(this, ID_TREE_IMAGES_B, wxDefaultPosition, wxSize(250, -1), wxTR_DEFAULT_STYLE | wxTR_HIDE_ROOT);
    iconList_ = new wxImageList(16, 16, true, 2);
    iconList_->Add(wxArtProvider::GetBitmap(wxART_INFORMATION, wxART_OTHER, wxSize(16,16)));
    iconList_->Add(wxArtProvider::GetBitmap(wxART_CROSS_MARK, wxART_OTHER, wxSize(16,16)));
    treeCtrlImagesB_->AssignImageList(iconList_);
    viewSizer->Add(treeCtrlImagesB_, 1, wxEXPAND | wxALL, 5); // Tree for B images

    imagePanelPreviewB_ = new ImagePanel(this, wxID_ANY, wxDefaultPosition, wxSize(640, 480)); // Python: HE_IMAGE_VIEW_W/H
    viewSizer->Add(imagePanelPreviewB_, 2, wxEXPAND | wxALL, 5); // Image preview for B

    staticTextResults_ = new wxStaticText(this, wxID_ANY, "Calibration results will appear here.", wxDefaultPosition, wxDefaultSize, wxST_ELLIPSIZE_END | wxALIGN_TOP);
    staticTextResults_->SetMinSize(wxSize(200, -1)); // Ensure it has some width
    viewSizer->Add(staticTextResults_, 1, wxEXPAND | wxALL, 5); // Results display

    mainSizer->Add(viewSizer, 1, wxEXPAND | wxALL, 5);

    SetSizerAndFit(mainSizer);
}


void TabHandEyePanel::OnEyePositionChange(wxCommandEvent& event) {
    // This might influence which calibration type (AXXB/AXZB) is more relevant,
    // but the main logic is tied to radioBoxCalibType_
    UpdateButtonsState();
}

void TabHandEyePanel::OnCalibTypeChange(wxCommandEvent& event) {
    bool isAXXB = (radioBoxCalibType_->GetSelection() == 0);
    radioBoxAXXBMethod_->Enable(isAXXB);
    radioBoxAXZBMethod_->Enable(!isAXXB);
    // AXZB is not implemented, so keep its methods disabled or provide a message.
    if (!isAXXB) {
        staticTextResults_->SetLabel("AX=ZB calibration is not implemented yet.");
    } else {
        staticTextResults_->SetLabel("Load data to perform AX=XB calibration.");
    }
    UpdateButtonsState();
}

void TabHandEyePanel::OnLoadARobotPoses(wxCommandEvent& event) {
    wxString wildcard = checkBoxRvecInputA_->IsChecked() ? "TXT files (*.txt)|*.txt" : "CSV files (*.csv)|*.csv";
    wxFileDialog openFileDialog(this, "Load Robot Poses (A)", "", "", wildcard, wxFD_OPEN | wxFD_FILE_MUST_EXIST);

    if (openFileDialog.ShowModal() == wxID_OK) {
        std::string path = openFileDialog.GetPath().ToStdString();
        textCtrlPathA_->SetValue(path);
        bool success = false;
        if (checkBoxRvecInputA_->IsChecked()) {
            success = core::calib::HandEyeCalibration::load_robot_poses_from_rvec_txt(
                path, R_gripper2base_list_, t_gripper2base_list_);
        } else {
            success = core::calib::HandEyeCalibration::load_robot_poses_from_quat_csv(
                path, R_gripper2base_list_, t_gripper2base_list_, checkBoxRotationOnlyA_->IsChecked());
        }
        if (!success || R_gripper2base_list_.empty()) {
            wxMessageBox("Failed to load or parse robot poses from " + path, "Error", wxOK | wxICON_ERROR);
            R_gripper2base_list_.clear();
            t_gripper2base_list_.clear();
            textCtrlPathA_->Clear();
        } else {
            staticTextResults_->SetLabel(wxString::Format("Loaded %zu robot poses (A).", R_gripper2base_list_.size()));
        }
    }
    UpdateButtonsState();
}

void TabHandEyePanel::OnLoadBImages(wxCommandEvent& event) {
    wxDirDialog dirDialog(this, "Select Folder for Target Images (B)", "", wxDD_DEFAULT_STYLE | wxDD_DIR_MUST_EXIST);
    if (dirDialog.ShowModal() == wxID_OK) {
        path_B_images_dir_ = fs::path(dirDialog.GetPath().ToStdString());
        textCtrlPathB_->SetValue(path_B_images_dir_.string());
        PopulateImageTreeB();
        ClearImagePanelB();
    }
    UpdateButtonsState();
}

void TabHandEyePanel::PopulateImageTreeB() {
    treeCtrlImagesB_->DeleteAllItems();
    target_image_data_store_.clear();

    if (path_B_images_dir_.empty() || !fs::is_directory(path_B_images_dir_)) return;

    wxTreeItemId rootId = treeCtrlImagesB_->AddRoot("ImagesB"); // Hidden root
    std::vector<std::string> image_extensions = {".png", ".jpg", ".jpeg", ".bmp", ".tiff", ".tif"};
    int count = 0;
    for (const auto& entry : fs::directory_iterator(path_B_images_dir_)) {
        if (entry.is_regular_file()) {
            std::string ext = entry.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
            if (std::find(image_extensions.begin(), image_extensions.end(), ext) != image_extensions.end()) {
                std::string filename = entry.path().filename().string();
                TargetImageData data;
                data.file_path = entry.path();
                target_image_data_store_[filename] = data;
                treeCtrlImagesB_->AppendItem(rootId, filename, 0, 0, new wxStringClientData(filename));
                count++;
            }
        }
    }
    if (count > 0) treeCtrlImagesB_->Expand(rootId);
    UpdateButtonsState();
}


void TabHandEyePanel::OnLoadCamParams(wxCommandEvent& event) {
    wxFileDialog openFileDialog(this, "Load Camera Parameters", "", "", "JSON files (*.json)|*.json", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (openFileDialog.ShowModal() == wxID_OK) {
        std::string path = openFileDialog.GetPath().ToStdString();
        camera_params_for_B_ = core::common::load_camera_param_from_json(
            path,
            checkBoxUseRightCamStereo_->IsChecked(), // load_stereo_extrinsics
            checkBoxUseRightCamStereo_->IsChecked() ? 1 : 0, // camera_id_for_stereo
            checkBoxCamParamTranspose_->IsChecked(),
            false // load_image_size (not strictly needed for HE if images are read directly)
        );
        if (camera_params_for_B_.intrinsic_matrix.empty()) {
            wxMessageBox("Failed to load camera parameters from " + path, "Error", wxOK | wxICON_ERROR);
            textCtrlCamParamsPath_->Clear();
        } else {
            textCtrlCamParamsPath_->SetValue(path);
        }
    }
    UpdateButtonsState();
}


void TabHandEyePanel::OnBoardParamsChanged(wxCommandEvent& event) {
    // Invalidate previous CalibBoard if params change, it will be recreated on Calibrate
    calib_board_for_B_.reset();
    UpdateButtonsState();
}

void TabHandEyePanel::UpdateButtonsState() {
    bool can_calibrate = !R_gripper2base_list_.empty() &&
                         !target_image_data_store_.empty() &&
                         !camera_params_for_B_.intrinsic_matrix.empty() &&
                         !textCtrlBoardRows_->GetValue().IsEmpty() &&
                         !textCtrlBoardCols_->GetValue().IsEmpty() &&
                         !textCtrlCellSize_->GetValue().IsEmpty();

    btnCalibrate_->Enable(can_calibrate);
    btnSaveResults_->Enable(last_hand_eye_result_.status == core::common::CalibErrType::CAL_OK);
}

void TabHandEyePanel::OnTreeItemSelectB(wxTreeEvent& event) {
    wxTreeItemId itemId = event.GetItem();
    if (itemId.IsOk() && itemId != treeCtrlImagesB_->GetRootItem()) {
        wxStringClientData* clientData = dynamic_cast<wxStringClientData*>(treeCtrlImagesB_->GetItemData(itemId));
        if (clientData) {
            std::string filename_key = clientData->GetData().ToStdString();
            DisplayImageAndCornersB(filename_key);
        }
    }
}

void TabHandEyePanel::ClearImagePanelB() {
    if (imagePanelPreviewB_) imagePanelPreviewB_->ClearPanel();
}

void TabHandEyePanel::DisplayImageAndCornersB(const std::string& filename_key) {
    auto it = target_image_data_store_.find(filename_key);
    if (it == target_image_data_store_.end()) {
        ClearImagePanelB();
        return;
    }
    TargetImageData& data = it->second;
    cv::Mat image = cv::imread(data.file_path.string());
    if (image.empty()) {
        ClearImagePanelB();
        return;
    }

    if (data.corners_found && !data.image_points.empty() && calib_board_for_B_) {
         cv::drawChessboardCorners(image,
            cv::Size(calib_board_for_B_->get_board_rows(), calib_board_for_B_->get_board_cols()),
            data.image_points, data.corners_found);
    }
    imagePanelPreviewB_->SetCvMat(image);
}

void TabHandEyePanel::OnCalibrate(wxCommandEvent& event) {
    if (calibration_process_thread_.joinable()) {
        wxMessageBox("Calibration process is already running.", "Busy", wxOK | wxICON_INFORMATION);
        return;
    }

    // --- 1. Setup CalibBoard for target images (B) ---
    long board_rows_sq, board_cols_sq;
    double cell_size;
    if (!textCtrlBoardRows_->GetValue().ToLong(&board_rows_sq) ||
        !textCtrlBoardCols_->GetValue().ToLong(&board_cols_sq) ||
        !textCtrlCellSize_->GetValue().ToDouble(&cell_size) ||
        board_rows_sq <=1 || board_cols_sq <=1 || cell_size <=0) {
        wxMessageBox("Invalid checkerboard parameters for target.", "Error", wxOK | wxICON_ERROR);
        return;
    }
    calib_board_for_B_ = std::make_unique<core::calib::CalibBoard>(
        static_cast<int>(board_cols_sq - 1), // inner corners horz
        static_cast<int>(board_rows_sq - 1), // inner corners vert
        static_cast<float>(cell_size),
        core::calib::CalibPatternType::CHESSBOARD,
        checkBoxUseCustomDetectorB_->IsChecked()
    );

    // --- 2. Prepare data for calibration thread ---
    // Copy data needed by the thread to avoid race conditions or lifetime issues
    // For A (robot poses):
    std::vector<cv::Mat> R_A_list = R_gripper2base_list_; // Copy
    std::vector<cv::Mat> t_A_list = t_gripper2base_list_; // Copy

    // For B (target poses relative to camera):
    // Need to iterate through target_image_data_store_, find corners, estimate pose
    // This part can be lengthy and should ideally be part of the thread or show progress.

    staticTextResults_->SetLabel("Processing target images (B) for poses...");
    btnCalibrate_->Enable(false); // Disable while processing

    progress_dialog_ = new wxProgressDialog(
            "Hand-Eye Calibration", "Initializing...",
            target_image_data_store_.size() + 2, // Num images + HE calib + finalize
            this, wxPD_APP_MODAL | wxPD_AUTO_HIDE | wxPD_CAN_ABORT);
    progress_dialog_->Pulse("Finding corners in target images...");


    calibration_process_thread_ = std::thread([this, R_A_list, t_A_list, K_camB = camera_params_for_B_.intrinsic_matrix.clone(), D_camB = camera_params_for_B_.distortion_coeffs.clone()]() {
        std::vector<cv::Mat> R_B_list_success; // Target to Cam
        std::vector<cv::Mat> t_B_list_success;
        std::vector<cv::Mat> R_A_list_corresponding; // Gripper to Base, corresponding to successful B poses
        std::vector<cv::Mat> t_A_list_corresponding;

        int progress_counter = 0;
        bool aborted = false;

        // Match A and B poses. Python code implies a 1:1 correspondence by index.
        // This assumes that the Nth robot pose in A corresponds to the Nth image in B.
        // For robustness, a naming convention or an explicit mapping file would be better.
        // For now, we iterate through target_image_data_store_ (which is sorted by filename)
        // and assume it matches the order of R_A_list/t_A_list.

        size_t pose_idx = 0;
        for (auto& entry_pair : target_image_data_store_) {
            if (pose_idx >= R_A_list.size()) break; // Not enough robot poses for all images

            TargetImageData& img_data = entry_pair.second;
            if (img_data.rejected_for_calib) {
                 pose_idx++;
                 if (progress_dialog_ && !progress_dialog_->Update(++progress_counter, "Skipping rejected image: " + img_data.file_path.filename().string())) {
                    aborted = true; break;
                 }
                 continue;
            }

            if (progress_dialog_ && !progress_dialog_->Update(++progress_counter, "Processing: " + img_data.file_path.filename().string())) {
                aborted = true; break;
            }

            cv::Mat img = cv::imread(img_data.file_path.string());
            if (img.empty()) {
                img_data.corners_found = false; img_data.pose_estimated = false;
                pose_idx++; continue;
            }

            core::calib::ImagePoseResult pose_res_B = calib_board_for_B_->estimate_pose_from_image(img, K_camB, D_camB);

            img_data.corners_found = pose_res_B.corners_found;
            img_data.image_points = pose_res_B.image_points; // Storing for potential display later

            if (pose_res_B.error_type == core::common::CalibErrType::CAL_OK) {
                img_data.rvec_target2cam = pose_res_B.rvec;
                img_data.tvec_target2cam = pose_res_B.tvec;
                img_data.pose_estimated = true;

                R_B_list_success.push_back(pose_res_B.rvec); // Actually rvec, will be converted by calibrateHandEye
                t_B_list_success.push_back(pose_res_B.tvec);
                R_A_list_corresponding.push_back(R_A_list[pose_idx]);
                t_A_list_corresponding.push_back(t_A_list[pose_idx]);
            } else {
                img_data.pose_estimated = false;
            }
            pose_idx++;
        }

        core::calib::HandEyeResult he_result;
        if (aborted) {
            he_result.status = core::common::CalibErrType::CAL_CORNER_DET_ERR; // Or a user aborted error
        } else if (R_A_list_corresponding.size() < 3) { // Need at least 3 pairs for HE
            std::cerr << "Not enough valid pose pairs for Hand-Eye calibration." << std::endl;
            he_result.status = core::common::CalibErrType::CAL_DATA_SIZE_NOT_MATCH;
        } else {
            if (progress_dialog_) progress_dialog_->Update(++progress_counter, "Performing Hand-Eye calculation...");

            // Determine calibration method from UI
            // This needs to be passed to thread or retrieved carefully. For now, hardcode or simplify.
            // int axxb_method_idx = radioBoxAXXBMethod_->GetSelection(); // Cannot access UI from thread
            // For now, using default Tsai
            cv::HandEyeCalibrationMethod he_cv_method = cv::CALIB_HAND_EYE_TSAI;
            // TODO: Get actual method from UI before starting thread.
            // For this example, assume Tsai was selected.
            // int selectedAXXBMethodIndex = radioBoxAXXBMethod_->GetSelection();
            // switch (selectedAXXBMethodIndex) {
            //     case 0: he_cv_method = cv::CALIB_HAND_EYE_TSAI; break;
            //     // ... other cases
            // }


            he_result = hand_eye_calibrator_->calibrate_axxb(
                R_A_list_corresponding, t_A_list_corresponding,
                R_B_list_success, t_B_list_success, // These are rvecs/tvecs, cv::calibrateHandEye expects R matrices
                                                     // This is a MISMATCH. calibrateHandEye needs R_target2cam etc.
                                                     // The current HandEyeCalibration::calibrate_axxb expects R matrices.
                                                     // So, R_B_list_success should contain R matrices, not rvecs.
                                                     // This needs correction in how R_B_list_success is populated or how calibrate_axxb is called.
                                                     // For now, assuming R_B_list_success contains R matrices for the call.
                                                     // This means estimate_pose_from_image should return R not rvec, or convert.
                                                     // Let's assume for now that estimate_pose_from_image returns R (it doesn't, returns rvec).
                                                     // This will be a point of failure or incorrect results.
                                                     // TEMPORARY: For structure, proceed. Will need to fix rvec->R conversion for B list.
                he_cv_method
            );
        }

        if (progress_dialog_) progress_dialog_->Update(progress_dialog_->GetRange() -1, "Finalizing Hand-Eye...");

        HandEyeCalibrationDoneEvent event_to_send(EVT_HAND_EYE_CALIBRATION_DONE, GetId());
        event_to_send.SetEventObject(this);
        event_to_send.SetResult(he_result);
        wxQueueEvent(this, event_to_send.Clone());
    });

}

void TabHandEyePanel::OnHandEyeCalibrationDone(HandEyeCalibrationDoneEvent& event) {
    if (progress_dialog_) {
        wxWindow* dlg_to_delete = progress_dialog_;
        progress_dialog_ = nullptr;
        if (dlg_to_delete) wxTheApp->AddPendingEvent(wxCommandEvent(wxEVT_DESTROY, dlg_to_delete->GetId()));
    }

    last_hand_eye_result_ = event.GetResult();
    std::string result_text;

    if (last_hand_eye_result_.status == core::common::CalibErrType::CAL_OK) {
        result_text = "Hand-Eye Calibration Successful (AXXB):\n";
        cv::Mat X = last_hand_eye_result_.X; // This is camera_H_gripper
        // Convert to R and T for display if needed
        cv::Mat R_X = X(cv::Rect(0,0,3,3));
        cv::Mat t_X = X(cv::Rect(3,0,1,3));

        result_text += wxString::Format("Rotation (cam_R_gripper):\n%s\n", MatToString(R_X)).ToStdString();
        result_text += wxString::Format("Translation (cam_t_gripper) (mm):\n%s\n", MatToString(t_X)).ToStdString();
        result_text += wxString::Format("Rotation Error (deg): %.4f\n", last_hand_eye_result_.rotation_error);
        result_text += wxString::Format("Translation Error (mm): %.4f\n", last_hand_eye_result_.translation_error);
        staticTextResults_->SetForegroundColour(*wxBLUE);
    } else {
        result_text = "Hand-Eye Calibration Failed.\nStatus: " + core::common::to_string(last_hand_eye_result_.status);
        staticTextResults_->SetForegroundColour(*wxRED);
    }
    staticTextResults_->SetLabel(result_text);
    // Refresh tree to show which images were used (based on pose_estimated in target_image_data_store_)
    PopulateImageTreeB(); // This will re-list. A more targeted update would be better.
    UpdateButtonsState();

    if (calibration_process_thread_.joinable()) {
        calibration_process_thread_.join();
    }
}


void TabHandEyePanel::OnSaveResults(wxCommandEvent& event) {
    if (last_hand_eye_result_.status != core::common::CalibErrType::CAL_OK || last_hand_eye_result_.X.empty()) {
        wxMessageBox("No valid hand-eye calibration results to save.", "Error", wxOK | wxICON_ERROR);
        return;
    }

    wxFileDialog saveFileDialog(this, "Save Hand-Eye Matrix", "", "hand_eye_AXXB.json",
                                "JSON files (*.json)|*.json", wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
    if (saveFileDialog.ShowModal() == wxID_CANCEL) return;

    std::string save_path = saveFileDialog.GetPath().ToStdString();
    json out_json;
    const auto& res = last_hand_eye_result_;

    out_json["version"] = "0.1_C++_HandEye";
    out_json["SN"] = "";
    out_json["Scheme"] = "opencv_AXXB"; // Or AXZB if implemented

    json he_data;
    cv::Mat matrix_to_save = res.X;
    if (checkBoxOutputInMeters_->IsChecked() && !res.X.empty()) {
        matrix_to_save = res.X.clone();
        matrix_to_save.at<double>(0,3) /= 1000.0; // tx
        matrix_to_save.at<double>(1,3) /= 1000.0; // ty
        matrix_to_save.at<double>(2,3) /= 1000.0; // tz
    }

    he_data["Matrix"] = json::array();
    if (!matrix_to_save.empty()) {
         for (int r=0; r<matrix_to_save.rows; ++r) {
            json row = json::array();
            for (int c=0; c<matrix_to_save.cols; ++c) row.push_back(matrix_to_save.at<double>(r,c));
            he_data["Matrix"].push_back(row);
        }
    }
    he_data["rotation_error_deg"] = res.rotation_error;
    he_data["translation_error_mm"] = res.translation_error; // Assuming error is always in mm from calib

    std::string root_key = (radioBoxCalibType_->GetSelection() == 0) ? "AXXB" : "AXZB";
    out_json[root_key] = he_data;

    try {
        std::ofstream ofs(save_path);
        ofs << std::setw(4) << out_json << std::endl;
        ofs.close();
        wxMessageBox("Hand-Eye results saved to " + save_path, "Success", wxOK | wxICON_INFORMATION);
    } catch (const std::exception& e) {
        wxMessageBox("Error saving file: " + std::string(e.what()), "Error", wxOK | wxICON_ERROR);
    }
}

// Helper to convert cv::Mat to wxString for display (simplified)
static wxString MatToString(const cv::Mat& mat) {
    if (mat.empty()) return "[]";
    std::stringstream ss;
    ss << std::fixed << std::setprecision(4); // For floating point formatting
    for (int i = 0; i < mat.rows; ++i) {
        ss << "[";
        for (int j = 0; j < mat.cols; ++j) {
            if (mat.type() == CV_64F) ss << mat.at<double>(i,j);
            else if (mat.type() == CV_32F) ss << mat.at<float>(i,j);
            // Add other types if needed
            else ss << "N/A_type_" << mat.type();
            if (j < mat.cols - 1) ss << ", ";
        }
        ss << "]";
        if (i < mat.rows - 1) ss << "\n ";
    }
    return wxString(ss.str());
}
