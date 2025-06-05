#ifndef TAB_STEREO_CAM_H
#define TAB_STEREO_CAM_H

#include <wx/wx.h>
#include <wx/panel.h>
#include <wx/treectrl.h>
#include <wx/button.h>
#include <wx/textctrl.h>
#include <wx/stattext.h>
#include <wx/checkbox.h>
#include <wx/progdlg.h>
#include <string>
#include <vector>
#include <filesystem>
#include <memory>
#include <thread>
#include <map>

#include <ui/components/image_panel.h>
#include <ui/components/custom_events.h> // For custom events (e.g. StereoCalibrationDoneEvent)
#include <ui/dialogs/stereo_file_loader_dialog.h> // For StereoFileLoaderDialog and StereoFileLoaderData
#include <core/calib/calib_board.h>
#include <core/calib/calib_types.h> // For StereoCalibResult

// Forward declaration for the event that carries stereo calibration results
class StereoCalibrationDoneEvent;
wxDECLARE_EVENT(EVT_STEREO_CALIBRATION_DONE, StereoCalibrationDoneEvent);

class StereoCalibrationDoneEvent : public wxCommandEvent {
public:
    StereoCalibrationDoneEvent(wxEventType eventType = EVT_STEREO_CALIBRATION_DONE, int id = 0)
        : wxCommandEvent(eventType, id) {}
    StereoCalibrationDoneEvent(const StereoCalibrationDoneEvent& event)
        : wxCommandEvent(event), result_(event.result_) {}

    virtual wxEvent* Clone() const override { return new StereoCalibrationDoneEvent(*this); }

    void SetResult(const core::calib::StereoCalibResult& res) { result_ = res; }
    const core::calib::StereoCalibResult& GetResult() const { return result_; }

private:
    core::calib::StereoCalibResult result_;
};

typedef void (wxEvtHandler::*StereoCalibrationDoneEventFunction)(StereoCalibrationDoneEvent&);
#define EVT_STEREO_CALIBRATION_DONE_HANDLER(id, fn) \
    wx__DECLARE_EVT1(EVT_STEREO_CALIBRATION_DONE, id, StereoCalibrationDoneEventFunction(fn))


class TabStereoCamPanel : public wxPanel {
public:
    TabStereoCamPanel(wxWindow* parent);
    ~TabStereoCamPanel();

private:
    // --- UI Elements ---
    wxTextCtrl* textCtrlLeftPath_;
    wxTextCtrl* textCtrlRightPath_;
    wxButton* btnOpenFileLoader_; // Opens StereoFileLoaderDialog

    wxCheckBox* checkBoxUseCustomDetector_; // For CalibBoard
    wxButton* btnCalibrate_;
    wxButton* btnSaveResults_;
    wxButton* btnShowDistribution_; // Placeholder
    wxStaticText* staticTextWarning_;

    wxTreeCtrl* treeCtrlImagePairs_;
    ImagePanel* imagePanelLeft_;
    ImagePanel* imagePanelRight_;
    wxStaticText* staticTextLeftName_;
    wxStaticText* staticTextRightName_;

    wxImageList* iconList_;

    // --- Event Handlers ---
    void OnOpenFileLoader(wxCommandEvent& event);
    void OnCalibrate(wxCommandEvent& event);
    void OnSaveResults(wxCommandEvent& event);
    void OnShowDistribution(wxCommandEvent& event); // Placeholder
    void OnTreeItemSelect(wxTreeEvent& event);
    void OnTreeItemRightClick(wxTreeEvent& event);
    void OnRecalibrateFromMenu(wxCommandEvent& event);
    void OnStereoCalibrationDone(StereoCalibrationDoneEvent& event);

    // --- Helper Methods ---
    void LayoutControls();
    void UpdateButtonsState();
    void PopulateImageTree(); // From image_pair_data_store_
    void DisplayImagePairAndCorners(const std::string& left_filename); // Uses map key (left_filename)
    std::vector<std::pair<std::filesystem::path, std::filesystem::path>> GetImagePathsForCalibration(bool include_rejected = false);
    void UpdateTreeWithCalibrationStatus(bool highlight_max_error = true);
    void ClearImagePanels();

    // --- State Variables ---
    StereoFileLoaderData current_stereo_params_; // Holds paths and board params from dialog
    std::unique_ptr<core::calib::CalibBoard> calib_board_;
    core::calib::StereoCalibResult last_stereo_calibration_result_;

    wxString tree_item_key_for_menu_; // Key (e.g. left filename) for right-click menu

    struct StereoImagePairData {
        std::filesystem::path left_file_path;
        std::filesystem::path right_file_path;
        bool rejected = false;
        double reprojection_error_left = -1.0;  // Or combined error if stereoCalibrate provides it per pair
        double reprojection_error_right = -1.0;
        std::vector<cv::Point2f> corners_left;
        std::vector<cv::Point2f> corners_right;
        bool corners_found_left = false;
        bool corners_found_right = false;
    };
    // Map key is the filename of the left image for simplicity
    std::map<std::string, StereoImagePairData> image_pair_data_store_;

    // Thread for calibration
    std::thread calibration_thread_;
    wxProgressDialog* progress_dialog_ = nullptr;

    enum ControlIds {
        ID_OPEN_FILE_LOADER_BTN = wxID_HIGHEST + 200,
        ID_CALIBRATE_BTN,
        ID_SAVE_BTN,
        ID_SHOW_DIST_BTN,
        ID_TREE_IMAGES,
        ID_RECALIBRATE_MENU_ITEM
    };
};

#endif // TAB_STEREO_CAM_H
