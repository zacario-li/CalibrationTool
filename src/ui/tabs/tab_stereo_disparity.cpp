#include "tab_stereo_disparity.h"
#include <wx/filedlg.h>
#include <wx/valnum.h> // For validators
#include <wx/artprov.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp> // For cv::reprojectImageTo3D
#include <nlohmann/json.hpp>   // For saving results (if needed, though Python only saved images)
#include <open3d/Open3D.h>    // For o3d::geometry::PointCloud and PCD I/O
#include <iostream>
#include <iomanip> // For std::setw, std::fixed for JSON
#include <fstream> // For std::ofstream

// VTK includes
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPolyData.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkProperty.h> // For actor->GetProperty()
#include <vtkGenericOpenGLRenderWindow.h> // Recommended for wxWidgets


// Define the event type for StereoProcessingDoneEvent
wxDEFINE_EVENT(EVT_STEREO_PROCESSING_DONE, StereoProcessingDoneEvent);

using json = nlohmann::json;
namespace fs = std::filesystem;

TabStereoDisparityPanel::TabStereoDisparityPanel(wxWindow* parent)
    : wxPanel(parent, wxID_ANY) {

    // Set default SGBM parameters (can be adjusted by UI)
    current_sgbm_params_.minDisparity = 0;
    current_sgbm_params_.numDisparities = 256;
    current_sgbm_params_.blockSize = 1; // Python default was 1, typical 3-11
    current_sgbm_params_.P1 = 1;        // Python default 1
    current_sgbm_params_.P2 = 128;      // Python default 128
    current_sgbm_params_.disp12MaxDiff = 1;
    current_sgbm_params_.preFilterCap = 15;
    current_sgbm_params_.uniquenessRatio = 5;
    current_sgbm_params_.speckleWindowSize = 50;
    current_sgbm_params_.speckleRange = 8;
    current_sgbm_params_.mode = cv::StereoSGBM::MODE_HH; // Python default MODE_HH (1)

    LayoutControls();

    // Bind events
    Bind(wxEVT_BUTTON, &TabStereoDisparityPanel::OnLoadCameraParams, this, ID_LOAD_CAM_PARAMS);
    Bind(wxEVT_BUTTON, &TabStereoDisparityPanel::OnLoadImages, this, ID_LOAD_IMAGES);
    Bind(wxEVT_BUTTON, &TabStereoDisparityPanel::OnComputeDisparity, this, ID_COMPUTE_DISPARITY);
    Bind(wxEVT_BUTTON, &TabStereoDisparityPanel::OnRectifyImages, this, ID_RECTIFY_IMAGES);
    Bind(wxEVT_BUTTON, &TabStereoDisparityPanel::OnComputeDepth, this, ID_COMPUTE_DEPTH);
    Bind(wxEVT_BUTTON, &TabStereoDisparityPanel::OnSaveResults, this, ID_SAVE_RESULTS);
    Bind(wxEVT_TREE_SEL_CHANGED, &TabStereoDisparityPanel::OnTreeItemSelect, this, ID_TREE_IMAGES);

    // Bind SGBM param changes
    Bind(wxEVT_RADIOBOX, &TabStereoDisparityPanel::OnSgbmParamChanged, this, ID_SGBM_MODE);
    Bind(wxEVT_TEXT_ENTER, &TabStereoDisparityPanel::OnSgbmParamChanged, this, ID_SGBM_BLOCKSIZE); // TEXT_ENTER to avoid too many updates
    Bind(wxEVT_TEXT_ENTER, &TabStereoDisparityPanel::OnSgbmParamChanged, this, ID_SGBM_P1);
    Bind(wxEVT_TEXT_ENTER, &TabStereoDisparityPanel::OnSgbmParamChanged, this, ID_SGBM_P2);
    Bind(wxEVT_TEXT_ENTER, &TabStereoDisparityPanel::OnSgbmParamChanged, this, ID_SGBM_MINDISP);
    Bind(wxEVT_TEXT_ENTER, &TabStereoDisparityPanel::OnSgbmParamChanged, this, ID_SGBM_NUMDISP);
    Bind(wxEVT_TEXT_ENTER, &TabStereoDisparityPanel::OnSgbmParamChanged, this, ID_SGBM_DISP12MAXDIFF);
    Bind(wxEVT_TEXT_ENTER, &TabStereoDisparityPanel::OnSgbmParamChanged, this, ID_SGBM_PREFILTERCAP);
    Bind(wxEVT_TEXT_ENTER, &TabStereoDisparityPanel::OnSgbmParamChanged, this, ID_SGBM_UNIQUENESS);
    Bind(wxEVT_TEXT_ENTER, &TabStereoDisparityPanel::OnSgbmParamChanged, this, ID_SGBM_SPECKLEWIN);
    Bind(wxEVT_TEXT_ENTER, &TabStereoDisparityPanel::OnSgbmParamChanged, this, ID_SGBM_SPECKLERANGE);
    // ZLimit is not an SGBM param, so no OnSgbmParamChanged for it.

    Bind(EVT_STEREO_PROCESSING_DONE_HANDLER(wxID_ANY, TabStereoDisparityPanel::OnStereoProcessingDone), this);

    UpdateButtonStates();
}

TabStereoDisparityPanel::~TabStereoDisparityPanel() {
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
    delete iconList_;
}

void TabStereoDisparityPanel::LayoutControls() {
    wxBoxSizer* mainSizer = new wxBoxSizer(wxVERTICAL);

    // --- SGBM Parameters UI ---
    wxStaticBoxSizer* sgbmParamsBox = new wxStaticBoxSizer(wxVERTICAL, this, "SGBM Parameters");

    wxArrayString sgbmModeChoices;
    sgbmModeChoices.Add("SGBM"); sgbmModeChoices.Add("HH"); sgbmModeChoices.Add("SGBM_3WAY"); sgbmModeChoices.Add("HH4");
    radioBoxSgbmMode_ = new wxRadioBox(sgbmParamsBox->GetStaticBox(), ID_SGBM_MODE, "Mode", wxDefaultPosition, wxDefaultSize, sgbmModeChoices, 1, wxRA_SPECIFY_ROWS);
    radioBoxSgbmMode_->SetSelection(1); // Default HH
    sgbmParamsBox->Add(radioBoxSgbmMode_, 0, wxALL, 5);

    wxFlexGridSizer* sgbmGrid = new wxFlexGridSizer(6, 4, 5, 5); // 6 params per row roughly, 4 columns (label, text, label, text)
    sgbmGrid->AddGrowableCol(1); sgbmGrid->AddGrowableCol(3);

    auto AddSgbmParam = [&](const wxString& label, wxWindowID id, const wxString& defaultValue, wxTextCtrl** ctrl) {
        sgbmGrid->Add(new wxStaticText(sgbmParamsBox->GetStaticBox(), wxID_ANY, label), 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT);
        *ctrl = new wxTextCtrl(sgbmParamsBox->GetStaticBox(), id, defaultValue, wxDefaultPosition, wxSize(60, -1));
        sgbmGrid->Add(*ctrl, 0, wxEXPAND);
    };

    AddSgbmParam("Block Size:", ID_SGBM_BLOCKSIZE, "1", &textCtrlSgbmBlockSize_);
    AddSgbmParam("P1:", ID_SGBM_P1, "1", &textCtrlSgbmP1_);
    AddSgbmParam("P2:", ID_SGBM_P2, "128", &textCtrlSgbmP2_);
    AddSgbmParam("Min Disparity:", ID_SGBM_MINDISP, "0", &textCtrlSgbmMinDisparity_);
    AddSgbmParam("Num Disparities:", ID_SGBM_NUMDISP, "256", &textCtrlSgbmNumDisparities_);
    AddSgbmParam("Disp12MaxDiff:", ID_SGBM_DISP12MAXDIFF, "1", &textCtrlSgbmDisp12MaxDiff_);
    AddSgbmParam("PreFilterCap:", ID_SGBM_PREFILTERCAP, "15", &textCtrlSgbmPreFilterCap_);
    AddSgbmParam("Uniqueness Ratio:", ID_SGBM_UNIQUENESS, "5", &textCtrlSgbmUniquenessRatio_);
    AddSgbmParam("Speckle Win Size:", ID_SGBM_SPECKLEWIN, "50", &textCtrlSgbmSpeckleWinSize_);
    AddSgbmParam("Speckle Range:", ID_SGBM_SPECKLERANGE, "8", &textCtrlSgbmSpeckleRange_);
    AddSgbmParam("Z Depth Limit (mm):", ID_SGBM_ZLIMIT, "5000", &textCtrlZLimitMm_); // Not an SGBM param, but used in depth processing
    sgbmGrid->AddSpacer(0); // Empty cell

    sgbmParamsBox->Add(sgbmGrid, 1, wxEXPAND | wxALL, 5);
    mainSizer->Add(sgbmParamsBox, 0, wxEXPAND | wxALL, 5);

    // --- Camera Params & Image Loading ---
    wxStaticBoxSizer* dataLoadBox = new wxStaticBoxSizer(wxHORIZONTAL, this, "Data Loading");
    textCtrlCameraParamsPath_ = new wxTextCtrl(dataLoadBox->GetStaticBox(), wxID_ANY, "", wxDefaultPosition, wxDefaultSize, wxTE_READONLY);
    btnLoadCameraParams_ = new wxButton(dataLoadBox->GetStaticBox(), ID_LOAD_CAM_PARAMS, "Load Stereo Params...");
    btnLoadImages_ = new wxButton(dataLoadBox->GetStaticBox(), ID_LOAD_IMAGES, "Load L/R Images...");
    dataLoadBox->Add(textCtrlCameraParamsPath_, 1, wxEXPAND | wxALIGN_CENTER_VERTICAL | wxALL, 5);
    dataLoadBox->Add(btnLoadCameraParams_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);
    dataLoadBox->Add(btnLoadImages_, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);
    mainSizer->Add(dataLoadBox, 0, wxEXPAND | wxALL, 5);

    // --- Operations ---
    wxStaticBoxSizer* opsBox = new wxStaticBoxSizer(wxHORIZONTAL, this, "Operations");
    btnComputeDisparity_ = new wxButton(opsBox->GetStaticBox(), ID_COMPUTE_DISPARITY, "Compute Disparity");
    btnRectifyImages_ = new wxButton(opsBox->GetStaticBox(), ID_RECTIFY_IMAGES, "Show Rectified");
    btnComputeDepth_ = new wxButton(opsBox->GetStaticBox(), ID_COMPUTE_DEPTH, "Compute Depth & Show 3D");
    btnSaveResults_ = new wxButton(opsBox->GetStaticBox(), ID_SAVE_RESULTS, "Save Results");
    opsBox->Add(btnComputeDisparity_, 0, wxALL, 5);
    opsBox->Add(btnRectifyImages_, 0, wxALL, 5);
    opsBox->Add(btnComputeDepth_, 0, wxALL, 5);
    opsBox->Add(btnSaveResults_, 0, wxALL, 5);
    mainSizer->Add(opsBox, 0, wxEXPAND | wxALL, 5);

    // --- Display Area ---
    wxBoxSizer* displaySizer = new wxBoxSizer(wxHORIZONTAL);
    treeCtrlImages_ = new wxTreeCtrl(this, ID_TREE_IMAGES, wxDefaultPosition, wxSize(250, -1), wxTR_DEFAULT_STYLE | wxTR_HIDE_ROOT);
    iconList_ = new wxImageList(16,16,true,1);
    iconList_->Add(wxArtProvider::GetBitmap(wxART_NORMAL_FILE, wxART_OTHER, wxSize(16,16)));
    treeCtrlImages_->AssignImageList(iconList_);
    displaySizer->Add(treeCtrlImages_, 1, wxEXPAND | wxALL, 5);

    // Python: DISP_IMAGE_VIEW_W = 576, DISP_IMAGE_VIEW_H = 384
    imagePanelLeftRawOrRect_ = new ImagePanel(this, wxID_ANY, wxDefaultPosition, wxSize(576, 384));
    imagePanelDisparity_ = new ImagePanel(this, wxID_ANY, wxDefaultPosition, wxSize(576, 384));
    displaySizer->Add(imagePanelLeftRawOrRect_, 2, wxEXPAND | wxALL, 5);
    displaySizer->Add(imagePanelDisparity_, 2, wxEXPAND | wxALL, 5);
    mainSizer->Add(displaySizer, 1, wxEXPAND | wxALL, 5);

    SetSizerAndFit(mainSizer);
}

void TabStereoDisparityPanel::OnLoadCameraParams(wxCommandEvent& event) {
    wxFileDialog openFileDialog(this, "Load Stereo Camera Parameters", "", "",
                                "JSON files (*.json)|*.json", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (openFileDialog.ShowModal() == wxID_OK) {
        camera_params_file_path_ = openFileDialog.GetPath().ToStdString();
        textCtrlCameraParamsPath_->SetValue(camera_params_file_path_);
        // Attempt to initialize SgbmProcessor if image size is known, or defer
        if (!input_image_size_.IsEmpty()) { // If images loaded first
            UpdateSgbmProcessorFromUI();
        }
    }
    UpdateButtonStates();
}

void TabStereoDisparityPanel::OnLoadImages(wxCommandEvent& event) {
    StereoFileLoaderData dialog_data; // Use default or last used paths
    dialog_data.left_folder_path = left_image_folder_path_;
    dialog_data.right_folder_path = right_image_folder_path_;
    // Board params are not used by this dialog in this tab, so defaults are fine.

    StereoFileLoaderDialog dialog(this, "Load Left and Right Image Folders", dialog_data);
    if (dialog.ShowModal() == wxID_OK) {
        const StereoFileLoaderData& new_data = dialog.GetData();
        left_image_folder_path_ = new_data.left_folder_path;
        right_image_folder_path_ = new_data.right_folder_path;
        PopulateImageTree();
        ClearImagePanels();
        if (!image_data_store_.empty() && input_image_size_.IsEmpty()) {
            // Get image size from first loaded image pair for SGBM init
            cv::Mat img = cv::imread(image_data_store_.begin()->second.left_image_path.string());
            if (!img.empty()) input_image_size_ = cv::Size(img.cols, img.rows);
        }
        if (!camera_params_file_path_.empty()) { // If params loaded first
             UpdateSgbmProcessorFromUI();
        }
    }
    UpdateButtonStates();
}

void TabStereoDisparityPanel::PopulateImageTree() {
    treeCtrlImages_->DeleteAllItems();
    image_data_store_.clear();
    selected_left_image_key_.clear();

    if (left_image_folder_path_.empty() || right_image_folder_path_.empty()) return;

    fs::path left_dir(left_image_folder_path_);
    fs::path right_dir(right_image_folder_path_);
    if (!fs::is_directory(left_dir) || !fs::is_directory(right_dir)) return;

    wxTreeItemId rootId = treeCtrlImages_->AddRoot("ImagePairs");
    std::vector<std::string> image_extensions = {".png", ".jpg", ".jpeg", ".bmp"};
    std::vector<fs::path> left_files, right_files;

    for (const auto& entry : fs::directory_iterator(left_dir)) {
        if (entry.is_regular_file() && std::find(image_extensions.begin(), image_extensions.end(), entry.path().extension().string()) != image_extensions.end()) {
            left_files.push_back(entry.path());
        }
    }
    for (const auto& entry : fs::directory_iterator(right_dir)) {
        if (entry.is_regular_file() && std::find(image_extensions.begin(), image_extensions.end(), entry.path().extension().string()) != image_extensions.end()) {
            right_files.push_back(entry.path());
        }
    }
    std::sort(left_files.begin(), left_files.end());
    std::sort(right_files.begin(), right_files.end());

    size_t num_pairs = std::min(left_files.size(), right_files.size());
    for (size_t i = 0; i < num_pairs; ++i) {
        std::string l_fname = left_files[i].filename().string();
        DisparityImageData data;
        data.left_image_path = left_files[i];
        data.right_image_path = right_files[i]; // Assuming pair by sorted order
        image_data_store_[l_fname] = data;
        treeCtrlImages_->AppendItem(rootId, l_fname, 0, 0, new wxStringClientData(l_fname));
    }
    if (num_pairs > 0) treeCtrlImages_->Expand(rootId);
    UpdateButtonStates();
}


void TabStereoDisparityPanel::OnSgbmParamChanged(wxCommandEvent& event) {
    UpdateSgbmProcessorFromUI();
}

void TabStereoDisparityPanel::UpdateSgbmProcessorFromUI() {
    if (camera_params_file_path_.empty() || input_image_size_.IsEmpty()) {
        // Not enough info to initialize SGBM processor yet
        return;
    }

    // Read SGBM params from UI
    current_sgbm_params_.mode = radioBoxSgbmMode_->GetSelection(); // Assumes order matches cv::StereoSGBM modes
    long val;
    if (textCtrlSgbmBlockSize_->GetValue().ToLong(&val)) current_sgbm_params_.blockSize = val;
    if (textCtrlSgbmP1_->GetValue().ToLong(&val)) current_sgbm_params_.P1 = val;
    if (textCtrlSgbmP2_->GetValue().ToLong(&val)) current_sgbm_params_.P2 = val;
    if (textCtrlSgbmMinDisparity_->GetValue().ToLong(&val)) current_sgbm_params_.minDisparity = val;
    if (textCtrlSgbmNumDisparities_->GetValue().ToLong(&val)) current_sgbm_params_.numDisparities = val;
    if (textCtrlSgbmDisp12MaxDiff_->GetValue().ToLong(&val)) current_sgbm_params_.disp12MaxDiff = val;
    if (textCtrlSgbmPreFilterCap_->GetValue().ToLong(&val)) current_sgbm_params_.preFilterCap = val;
    if (textCtrlSgbmUniquenessRatio_->GetValue().ToLong(&val)) current_sgbm_params_.uniquenessRatio = val;
    if (textCtrlSgbmSpeckleWinSize_->GetValue().ToLong(&val)) current_sgbm_params_.speckleWindowSize = val;
    if (textCtrlSgbmSpeckleRange_->GetValue().ToLong(&val)) current_sgbm_params_.speckleRange = val;

    current_sgbm_params_.validate(); // Ensure values are consistent (e.g. numDisparities % 16 == 0)

    // Update UI with validated values (if they changed)
    textCtrlSgbmNumDisparities_->SetValue(std::to_string(current_sgbm_params_.numDisparities));
    textCtrlSgbmBlockSize_->SetValue(std::to_string(current_sgbm_params_.blockSize));


    if (!sgbm_processor_ || !sgbm_processor_->is_initialized()) {
        sgbm_processor_ = std::make_unique<core::depth::SgbmProcessor>();
        if (!sgbm_processor_->initialize(camera_params_file_path_, current_sgbm_params_, input_image_size_)) {
            wxMessageBox("Failed to initialize SGBM Processor. Check camera params and image size.", "Error", wxOK | wxICON_ERROR);
            sgbm_processor_.reset();
        }
    } else {
        sgbm_processor_->update_sgbm_parameters(current_sgbm_params_);
    }
    UpdateButtonStates();
}

void TabStereoDisparityPanel::UpdateButtonStates() {
    bool cam_params_loaded = !camera_params_file_path_.empty() && sgbm_processor_ && sgbm_processor_->is_initialized();
    bool images_available = !image_data_store_.empty() && !selected_left_image_key_.empty();

    btnComputeDisparity_->Enable(cam_params_loaded && images_available);
    btnRectifyImages_->Enable(cam_params_loaded && images_available && image_data_store_.count(selected_left_image_key_) && image_data_store_[selected_left_image_key_].is_processed);
    btnComputeDepth_->Enable(cam_params_loaded && images_available && image_data_store_.count(selected_left_image_key_) && image_data_store_[selected_left_image_key_].is_processed);
    btnSaveResults_->Enable(images_available && image_data_store_.count(selected_left_image_key_) && image_data_store_[selected_left_image_key_].is_processed);
}

void TabStereoDisparityPanel::ClearImagePanels() {
    imagePanelLeftRawOrRect_->ClearPanel();
    imagePanelDisparity_->ClearPanel();
}

void TabStereoDisparityPanel::OnTreeItemSelect(wxTreeEvent& event) {
    wxTreeItemId itemId = event.GetItem();
    if (itemId.IsOk() && itemId != treeCtrlImages_->GetRootItem()) {
        wxStringClientData* clientData = dynamic_cast<wxStringClientData*>(treeCtrlImages_->GetItemData(itemId));
        if (clientData) {
            selected_left_image_key_ = clientData->GetData().ToStdString();
            auto it = image_data_store_.find(selected_left_image_key_);
            if (it != image_data_store_.end()) {
                DisparityImageData& data = it->second;
                cv::Mat left_display_img = data.is_processed ? data.left_rectified_img : cv::imread(data.left_image_path.string());
                imagePanelLeftRawOrRect_->SetCvMat(left_display_img);
                if (data.is_processed && !data.disparity_map.empty()) {
                    cv::Mat disp_vis;
                    cv::normalize(data.disparity_map, disp_vis, 0, 255, cv::NORM_MINMAX, CV_8U);
                    cv::cvtColor(disp_vis, disp_vis, cv::COLOR_GRAY2BGR); // ImagePanel expects BGR or Gray
                    imagePanelDisparity_->SetCvMat(disp_vis);
                } else {
                    imagePanelDisparity_->ClearPanel();
                }
            }
        }
    }
    UpdateButtonStates();
}


void TabStereoDisparityPanel::OnComputeDisparity(wxCommandEvent& event) {
    if (selected_left_image_key_.empty() || !sgbm_processor_ || !sgbm_processor_->is_initialized()) {
        wxMessageBox("Please select an image pair and ensure camera parameters are loaded.", "Error", wxOK | wxICON_ERROR);
        return;
    }
    if (processing_thread_.joinable()) {
        wxMessageBox("A stereo process is already running.", "Busy", wxOK | wxICON_INFORMATION);
        return;
    }

    auto it = image_data_store_.find(selected_left_image_key_);
    if (it == image_data_store_.end()) return;

    DisparityImageData& current_pair_data = it->second; // Get a reference

    btnComputeDisparity_->Enable(false); // Disable button during processing
    progress_dialog_ = new wxProgressDialog("Disparity Computation", "Processing...", 100, this, wxPD_APP_MODAL | wxPD_AUTO_HIDE | wxPD_CAN_ABORT);

    processing_thread_ = std::thread([this, &current_pair_data]() { // Pass data by reference carefully
        cv::Mat left_raw = cv::imread(current_pair_data.left_image_path.string());
        cv::Mat right_raw = cv::imread(current_pair_data.right_image_path.string());

        if (left_raw.empty() || right_raw.empty()) {
            wxLogMessage("Error loading images for disparity."); // Use wxLog for thread-safe messages
            // Post an event to notify UI of failure
            StereoProcessingDoneEvent result_event(EVT_STEREO_PROCESSING_DONE, GetId());
            result_event.SetProcessingType(StereoProcessingType::DISPARITY); // Indicate failure state if possible
            wxQueueEvent(this, result_event.Clone());
            return;
        }

        // Ensure image size matches what SGBM processor was initialized with
        if (left_raw.size() != input_image_size_) {
             cv::resize(left_raw, left_raw, input_image_size_);
             cv::resize(right_raw, right_raw, input_image_size_);
        }


        if (progress_dialog_) progress_dialog_->Update(20, "Rectifying images...");
        if (progress_dialog_ && progress_dialog_->WasCancelled()) { /* handle abort */ return; }

        sgbm_processor_->rectify_images(left_raw, right_raw, current_pair_data.left_rectified_img, current_pair_data.right_rectified_img);

        if (progress_dialog_) progress_dialog_->Update(50, "Computing disparity...");
        if (progress_dialog_ && progress_dialog_->WasCancelled()) { /* handle abort */ return; }

        cv::Mat left_gray, right_gray;
        cv::cvtColor(current_pair_data.left_rectified_img, left_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(current_pair_data.right_rectified_img, right_gray, cv::COLOR_BGR2GRAY);

        current_pair_data.disparity_map = sgbm_processor_->compute_disparity(left_gray, right_gray);
        current_pair_data.is_processed = !current_pair_data.disparity_map.empty();

        if (progress_dialog_) progress_dialog_->Update(90, "Finalizing disparity...");

        StereoProcessingDoneEvent result_event(EVT_STEREO_PROCESSING_DONE, GetId());
        result_event.SetProcessingType(StereoProcessingType::DISPARITY);
        // No need to SetDisparityMap on event if main thread accesses member data
        // result_event.SetDisparityMap(current_pair_data.disparity_map);
        wxQueueEvent(this, result_event.Clone());
    });
}

void TabStereoDisparityPanel::OnStereoProcessingDone(StereoProcessingDoneEvent& event) {
    if (progress_dialog_) {
        wxWindow* dlg_to_delete = progress_dialog_;
        progress_dialog_ = nullptr;
        if (dlg_to_delete) wxTheApp->AddPendingEvent(wxCommandEvent(wxEVT_DESTROY, dlg_to_delete->GetId()));
    }

    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }

    // Refresh UI based on selected_left_image_key_ which should hold the processed image's data
    auto it = image_data_store_.find(selected_left_image_key_);
    if (it != image_data_store_.end()) {
        DisparityImageData& data = it->second;
        if (event.GetProcessingType() == StereoProcessingType::DISPARITY) {
            if (data.is_processed) {
                imagePanelLeftRawOrRect_->SetCvMat(data.left_rectified_img);
                cv::Mat disp_vis;
                cv::normalize(data.disparity_map, disp_vis, 0, 255, cv::NORM_MINMAX, CV_8U);
                cv::cvtColor(disp_vis, disp_vis, cv::COLOR_GRAY2BGR);
                imagePanelDisparity_->SetCvMat(disp_vis);
                wxLogStatus("Disparity computation complete for " + selected_left_image_key_);
            } else {
                wxLogError("Disparity computation failed for " + selected_left_image_key_);
                imagePanelDisparity_->ClearPanel();
            }
        } else if (event.GetProcessingType() == StereoProcessingType::DEPTH_AND_VISUALIZE) {
            // This type implies point cloud was generated and possibly saved to data.pcd_file_path
            // The VTK visualization part is synchronous for now.
            if (!data.pcd_file_path.empty()) {
                 wxLogStatus("Depth computation complete. Point cloud saved to: " + data.pcd_file_path);
            } else {
                 wxLogError("Depth computation or point cloud saving failed.");
            }
        }
    }
    UpdateButtonStates(); // Re-enable buttons
}


void TabStereoDisparityPanel::OnRectifyImages(wxCommandEvent& event) {
    if (selected_left_image_key_.empty()) return;
    auto it = image_data_store_.find(selected_left_image_key_);
    if (it != image_data_store_.end() && it->second.is_processed) {
        DisparityImageData& data = it->second;
        cv::Mat combined_rectified;
        if (data.left_rectified_img.empty() || data.right_rectified_img.empty()) {
            wxMessageBox("Rectified images not available. Compute disparity first.", "Info", wxOK | wxICON_INFORMATION);
            return;
        }

        cv::Mat l_rect = data.left_rectified_img.clone();
        cv::Mat r_rect = data.right_rect_img.clone();

        // Draw epipolar lines for visualization
        int num_lines = 10;
        for (int i = 1; i <= num_lines; ++i) {
            int y = l_rect.rows * i / (num_lines + 1);
            cv::line(l_rect, cv::Point(0, y), cv::Point(l_rect.cols - 1, y), cv::Scalar(0, 255, 0), 1);
            cv::line(r_rect, cv::Point(0, y), cv::Point(r_rect.cols - 1, y), cv::Scalar(0, 255, 0), 1);
        }
        cv::hconcat(l_rect, r_rect, combined_rectified);

        // Show in a new OpenCV window or a DetailsImagePanel
        // For simplicity, using OpenCV window for now
        if (!combined_rectified.empty()) {
            cv::imshow("Rectified Stereo Pair with Epipolar Lines", combined_rectified);
            cv::waitKey(0); // User needs to close this window
        }
    } else {
        wxMessageBox("No processed image selected or disparity not computed yet.", "Info", wxOK | wxICON_INFORMATION);
    }
}

void TabStereoDisparityPanel::OnComputeDepth(wxCommandEvent& event) {
    if (selected_left_image_key_.empty() || !sgbm_processor_ || !sgbm_processor_->is_initialized()) {
        wxMessageBox("Select processed image pair & ensure camera params are loaded.", "Error", wxOK | wxICON_ERROR);
        return;
    }
    auto it = image_data_store_.find(selected_left_image_key_);
    if (it == image_data_store_.end() || !it->second.is_processed || it->second.disparity_map.empty()) {
        wxMessageBox("Disparity map not available for selected image. Compute disparity first.", "Error", wxOK | wxICON_ERROR);
        return;
    }

    DisparityImageData& current_pair_data = it->second;

    if (processing_thread_.joinable()) {
        wxMessageBox("A stereo process is already running.", "Busy", wxOK | wxICON_INFORMATION);
        return;
    }

    btnComputeDepth_->Enable(false);
    progress_dialog_ = new wxProgressDialog("Depth Computation", "Processing depth...", 100, this, wxPD_APP_MODAL | wxPD_AUTO_HIDE | wxPD_CAN_ABORT);

    processing_thread_ = std::thread([this, &current_pair_data]() {
        cv::Mat disparity_float;
        current_pair_data.disparity_map.convertTo(disparity_float, CV_32F, 1.0 / 16.0); // SGBM output is 16*disp

        if (progress_dialog_) progress_dialog_->Update(20, "Reprojecting to 3D...");
        if (progress_dialog_ && progress_dialog_->WasCancelled()) { /* handle abort */ return; }

        cv::Mat points_3d_cv;
        cv::reprojectImageTo3D(disparity_float, points_3d_cv, sgbm_processor_->get_q_matrix(), true, CV_32F);

        if (progress_dialog_) progress_dialog_->Update(50, "Filtering points...");
        if (progress_dialog_ && progress_dialog_->WasCancelled()) { /* handle abort */ return; }

        vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkUnsignedCharArray> vtk_colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        vtk_colors->SetNumberOfComponents(3);
        vtk_colors->SetName("Colors");

        cv::Mat color_source = current_pair_data.left_rectified_img; // Use left rectified for color
        if (color_source.empty()) {
            wxLogError("Left rectified image is empty for coloring point cloud.");
             StereoProcessingDoneEvent result_event(EVT_STEREO_PROCESSING_DONE, GetId());
            result_event.SetProcessingType(StereoProcessingType::DEPTH_AND_VISUALIZE);
            wxQueueEvent(this, result_event.Clone());
            return;
        }
        if(color_source.type() != CV_8UC3) cv::cvtColor(color_source, color_source, cv::COLOR_GRAY2BGR);


        double z_limit_mm = 5000.0; // Default
        textCtrlZLimitMm_->GetValue().ToDouble(&z_limit_mm);

        for (int r = 0; r < points_3d_cv.rows; ++r) {
            for (int c = 0; c < points_3d_cv.cols; ++c) {
                cv::Vec3f p = points_3d_cv.at<cv::Vec3f>(r, c);
                // Filter out invalid points (e.g., large Z, or where disparity was 0 or minDisparity-1)
                if (std::isfinite(p[2]) && p[2] > 0 && p[2] < z_limit_mm) { // Z is in mm from Q matrix
                    vtk_points->InsertNextPoint(p[0], -p[1], -p[2]); // VTK: Y up, Z into screen (if camera looks along +Z)
                                                                    // OpenCV: Y down, Z out of screen
                                                                    // Negating Y and Z for typical VTK view.
                    cv::Vec3b color = color_source.at<cv::Vec3b>(r, c);
                    vtk_colors->InsertNextTuple3(color[0], color[1], color[2]); // BGR -> RGB if needed by VTK or shader
                }
            }
        }

        // Save to PCD for this pair
        std::string pcd_filename = "pointcloud_" + fs::path(current_pair_data.left_image_path).stem().string() + ".pcd";
        fs::path pcd_full_path = fs::temp_directory_path() / pcd_filename; // Save to temp for now

        if (vtk_points->GetNumberOfPoints() > 0) {
            o3d::geometry::PointCloud o3d_pcd;
            std::vector<Eigen::Vector3d> o3d_points_vec;
            std::vector<Eigen::Vector3d> o3d_colors_vec;
            o3d_points_vec.reserve(vtk_points->GetNumberOfPoints());
            o3d_colors_vec.reserve(vtk_points->GetNumberOfPoints());

            for (vtkIdType i = 0; i < vtk_points->GetNumberOfPoints(); ++i) {
                double pt[3];
                vtk_points->GetPoint(i, pt);
                o3d_points_vec.push_back(Eigen::Vector3d(pt[0], pt[1], pt[2]));

                unsigned char color[3];
                vtk_colors->GetTypedTuple(i, color);
                o3d_colors_vec.push_back(Eigen::Vector3d(color[0]/255.0, color[1]/255.0, color[2]/255.0));
            }
            o3d_pcd.SetPoints(o3d_points_vec);
            o3d_pcd.SetColors(o3d_colors_vec);
            o3d::io::WritePointCloud(pcd_full_path.string(), o3d_pcd);
            current_pair_data.pcd_file_path = pcd_full_path.string();
        } else {
            current_pair_data.pcd_file_path.clear();
        }


        if (progress_dialog_) progress_dialog_->Update(90, "Finalizing depth...");

        // For VTK visualization, it's better done in the main thread.
        // Signal completion and let main thread call ShowPointCloudVTK.
        StereoProcessingDoneEvent result_event(EVT_STEREO_PROCESSING_DONE, GetId());
        result_event.SetProcessingType(StereoProcessingType::DEPTH_AND_VISUALIZE);
        // Store vtk_points and vtk_colors as members if needed by ShowPointCloudVTK, or pass path
        wxQueueEvent(this, result_event.Clone());

        // Synchronous call to VTK (can freeze UI if long)
        // ShowPointCloudVTK(vtk_points, vtk_colors); // This would be bad from worker thread
    });
}

void TabStereoDisparityPanel::ShowPointCloudVTK(vtkSmartPointer<vtkPoints> points, vtkSmartPointer<vtkUnsignedCharArray> colors) {
    if (!points || points->GetNumberOfPoints() == 0) {
        wxMessageBox("No points to display in 3D.", "Info", wxOK | wxICON_INFORMATION);
        return;
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    if (colors && colors->GetNumberOfTuples() == points->GetNumberOfPoints()) {
        polyData->GetPointData()->SetScalars(colors);
    }

    vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    vertexFilter->SetInputData(polyData);
    vertexFilter->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(vertexFilter->GetOutputPort());
    if (colors) { // Enable scalar visibility if colors are present
        mapper->ScalarVisibilityOn();
        mapper->SetScalarModeToUsePointData();
    }


    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(2); // Adjust point size

    // Create a new render window for each call or reuse one. Reusing is complex.
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor(actor);
    renderer->SetBackground(0.1, 0.2, 0.4); // Background color

    // Using vtkGenericOpenGLRenderWindow for better wxWidgets compatibility
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    renderWindow->SetWindowName("3D Point Cloud Viewer");
    renderWindow->SetSize(800, 600);

    vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetRenderWindow(renderWindow);

    vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    interactor->SetInteractorStyle(style);

    renderWindow->Render();
    interactor->Initialize();
    interactor->Start(); // This blocks until window is closed.
    // For non-blocking, need to integrate into wx event loop (e.g. wxVTKRenderWindowInteractor)
}


void TabStereoDisparityPanel::OnSaveResults(wxCommandEvent& event) {
    if (selected_left_image_key_.empty()) {
        wxMessageBox("Please select an image pair first.", "Info", wxOK | wxICON_INFORMATION);
        return;
    }
    auto it = image_data_store_.find(selected_left_image_key_);
    if (it == image_data_store_.end() || !it->second.is_processed) {
        wxMessageBox("No processed data available for the selected image pair.", "Info", wxOK | wxICON_INFORMATION);
        return;
    }
    const DisparityImageData& data = it->second;

    wxFileDialog saveFileDialog(this, "Save Results", "", selected_left_image_key_,
                                "PNG files (*.png)|*.png|PCD files (*.pcd)|*.pcd",
                                wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
    if (saveFileDialog.ShowModal() == wxID_CANCEL) return;

    std::string save_path_str = saveFileDialog.GetPath().ToStdString();
    fs::path save_path(save_path_str);
    std::string ext = save_path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    bool success = false;
    if (ext == ".png" && !data.disparity_map.empty()) {
        cv::Mat disp_vis;
        cv::normalize(data.disparity_map, disp_vis, 0, 255, cv::NORM_MINMAX, CV_8U);
        success = cv::imwrite(save_path_str, disp_vis);
    } else if (ext == ".pcd" && !data.pcd_file_path.empty() && fs::exists(data.pcd_file_path)) {
        try {
            // Copy the temp PCD file to the desired save location
            fs::copy_file(data.pcd_file_path, save_path, fs::copy_options::overwrite_existing);
            success = true;
        } catch (const fs::filesystem_error& e) {
            wxLogError("Failed to copy PCD file: %s", e.what());
            success = false;
        }
    } else {
        wxMessageBox("Selected file type cannot be saved or data is missing.", "Error", wxOK | wxICON_ERROR);
        return;
    }

    if (success) {
        wxMessageBox("File saved successfully to " + save_path_str, "Success", wxOK | wxICON_INFORMATION);
    } else {
        wxMessageBox("Failed to save file to " + save_path_str, "Error", wxOK | wxICON_ERROR);
    }
}
