#ifndef TAB_HAND_EYE_H
#define TAB_HAND_EYE_H

#include <wx/wx.h>
#include <wx/panel.h>
#include <wx/radiobox.h>
#include <wx/checkbox.h>
#include <wx/textctrl.h>
#include <wx/button.h>
#include <wx/treectrl.h>
#include <wx/stattext.h>
#include <wx/progdlg.h>
#include <string>
#include <vector>
#include <filesystem>
#include <memory>
#include <thread>
#include <map>

#include <ui/components/image_panel.h>
#include <ui/components/custom_events.h> // For custom events
#include <core/calib/calib_board.h>
#include <core/calib/hand_eye_calibration.h>
#include <core/calib/calib_types.h> // For HandEyeResult, CameraParams, ImageCornersResult
#include <core/common/file_utils.h> // For load_camera_param_from_json

// Forward declaration for the event that carries hand-eye calibration results
class HandEyeCalibrationDoneEvent;
wxDECLARE_EVENT(EVT_HAND_EYE_CALIBRATION_DONE, HandEyeCalibrationDoneEvent);

class HandEyeCalibrationDoneEvent : public wxCommandEvent {
public:
    HandEyeCalibrationDoneEvent(wxEventType eventType = EVT_HAND_EYE_CALIBRATION_DONE, int id = 0)
        : wxCommandEvent(eventType, id) {}
    HandEyeCalibrationDoneEvent(const HandEyeCalibrationDoneEvent& event)
        : wxCommandEvent(event), result_(event.result_) {}

    virtual wxEvent* Clone() const override { return new HandEyeCalibrationDoneEvent(*this); }

    void SetResult(const core::calib::HandEyeResult& res) { result_ = res; }
    const core::calib::HandEyeResult& GetResult() const { return result_; }

private:
    core::calib::HandEyeResult result_;
};

typedef void (wxEvtHandler::*HandEyeCalibrationDoneEventFunction)(HandEyeCalibrationDoneEvent&);
#define EVT_HAND_EYE_CALIBRATION_DONE_HANDLER(id, fn) \
    wx__DECLARE_EVT1(EVT_HAND_EYE_CALIBRATION_DONE, id, HandEyeCalibrationDoneEventFunction(fn))


class TabHandEyePanel : public wxPanel {
public:
    TabHandEyePanel(wxWindow* parent);
    ~TabHandEyePanel();

private:
    // --- UI Elements ---
    wxRadioBox* radioBoxEyePosition_; // Eye in Hand (AXXB), Eye to Hand (AXZB)
    wxRadioBox* radioBoxCalibType_;   // AXXB, AXZB (though only AXXB implemented)
    wxCheckBox* checkBoxRotationOnlyA_; // For robot poses (A)
    wxCheckBox* checkBoxRvecInputA_;    // If A is rvec txt file

    wxRadioBox* radioBoxAXXBMethod_;
    wxRadioBox* radioBoxAXZBMethod_;

    wxTextCtrl* textCtrlPathA_; // Robot poses (gripper_H_base)
    wxButton* btnLoadA_;
    wxTextCtrl* textCtrlPathB_; // Target images (target_H_camera to be derived)
    wxButton* btnLoadB_;
    wxTextCtrl* textCtrlCamParamsPath_;
    wxButton* btnLoadCamParams_;
    wxCheckBox* checkBoxCamParamTranspose_; // If intrinsic matrix needs transpose
    wxCheckBox* checkBoxUseRightCamStereo_; // If stereo params, use cam2

    wxTextCtrl* textCtrlBoardRows_; // Squares
    wxTextCtrl* textCtrlBoardCols_; // Squares
    wxTextCtrl* textCtrlCellSize_;
    wxCheckBox* checkBoxUseCustomDetectorB_; // For target images (B)
    wxCheckBox* checkBoxOutputInMeters_; // For saving result

    wxButton* btnCalibrate_;
    wxButton* btnSaveResults_;

    wxTreeCtrl* treeCtrlImagesB_; // Lists images for target_H_camera poses
    ImagePanel* imagePanelPreviewB_;
    wxStaticText* staticTextResults_; // Display R_err, T_err and matrix

    wxImageList* iconList_;


    // --- Event Handlers ---
    void OnEyePositionChange(wxCommandEvent& event);
    void OnCalibTypeChange(wxCommandEvent& event); // AXXB vs AXZB
    void OnLoadARobotPoses(wxCommandEvent& event);
    void OnLoadBImages(wxCommandEvent& event);
    void OnLoadCamParams(wxCommandEvent& event);
    void OnCalibrate(wxCommandEvent& event);
    void OnSaveResults(wxCommandEvent& event);
    void OnTreeItemSelectB(wxTreeEvent& event);
    void OnBoardParamsChanged(wxCommandEvent& event); // For textCtrlRows_, Cols_, CellSize_
    void OnHandEyeCalibrationDone(HandEyeCalibrationDoneEvent& event);
    // void OnCheckBoxChanged(wxCommandEvent& event); // Generic for enabling/disabling

    // --- Helper Methods ---
    void LayoutControls();
    void UpdateButtonsState();
    void PopulateImageTreeB();
    void DisplayImageAndCornersB(const std::string& filename_key);
    void ClearImagePanelB();

    // --- State Variables ---
    std::unique_ptr<core::calib::HandEyeCalibration> hand_eye_calibrator_;
    std::unique_ptr<core::calib::CalibBoard> calib_board_for_B_; // For finding target poses

    // Loaded Data
    std::vector<cv::Mat> R_gripper2base_list_; // Robot Poses (A)
    std::vector<cv::Mat> t_gripper2base_list_;
    core::common::CameraParams camera_params_for_B_; // Camera parameters for observing target
    std::filesystem::path path_B_images_dir_;

    struct TargetImageData {
        std::filesystem::path file_path;
        bool corners_found = false;
        std::vector<cv::Point2f> image_points;
        cv::Mat rvec_target2cam; // (R_B)
        cv::Mat tvec_target2cam; // (t_B)
        bool pose_estimated = false;
        bool rejected_for_calib = false; // User or auto rejected
    };
    std::map<std::string, TargetImageData> target_image_data_store_; // Key: filename

    core::calib::HandEyeResult last_hand_eye_result_;

    // Threading
    std::thread calibration_process_thread_; // For the entire process (corner finding + HE calib)
    wxProgressDialog* progress_dialog_ = nullptr;

    enum ControlIds {
        ID_LOAD_A_BTN = wxID_HIGHEST + 300,
        ID_LOAD_B_BTN,
        ID_LOAD_CAM_PARAMS_BTN,
        ID_CALIBRATE_BTN,
        ID_SAVE_BTN,
        ID_TREE_IMAGES_B,
        ID_BOARD_ROWS_TEXT,
        ID_BOARD_COLS_TEXT,
        ID_BOARD_CELLSIZE_TEXT,
        // Add other specific IDs if needed
    };
};

#endif // TAB_HAND_EYE_H
