#ifndef TAB_SINGLE_CAM_H
#define TAB_SINGLE_CAM_H

#include <wx/wx.h>
#include <wx/panel.h>
#include <wx/treectrl.h>
#include <wx/button.h>
#include <wx/textctrl.h>
#include <wx/stattext.h>
#include <wx/checkbox.h>
#include <wx/progdlg.h> // For wxProgressDialog
#include <string>
#include <vector>
#include <filesystem>
#include <memory> // For std::unique_ptr
#include <thread> // For std::thread
#include <map> // For image_data_store_

#include <ui/components/image_panel.h>
#include <ui/components/custom_events.h> // For custom events
#include <core/calib/calib_board.h>    // For CalibBoard
#include <core/calib/calib_types.h>    // For MonoCalibResult, ImageCornersResult
// #include <core/common/storage_service.h> // Stubbed for now

// Forward declaration for the event that carries calibration results
class CalibrationDoneEvent;
wxDECLARE_EVENT(EVT_CALIBRATION_DONE, CalibrationDoneEvent); // Custom event type

class CalibrationDoneEvent : public wxCommandEvent {
public:
    CalibrationDoneEvent(wxEventType eventType = EVT_CALIBRATION_DONE, int id = 0)
        : wxCommandEvent(eventType, id) {}
    CalibrationDoneEvent(const CalibrationDoneEvent& event)
        : wxCommandEvent(event), result_(event.result_) {} // Copy constructor

    virtual wxEvent* Clone() const override { return new CalibrationDoneEvent(*this); }

    void SetResult(const core::calib::MonoCalibResult& res) { result_ = res; }
    const core::calib::MonoCalibResult& GetResult() const { return result_; }

private:
    core::calib::MonoCalibResult result_;
};

typedef void (wxEvtHandler::*CalibrationDoneEventFunction)(CalibrationDoneEvent&);
#define EVT_CALIBRATION_DONE_HANDLER(id, fn) \
    wx__DECLARE_EVT1(EVT_CALIBRATION_DONE, id, CalibrationDoneEventFunction(fn))


class TabSingleCamPanel : public wxPanel {
public:
    TabSingleCamPanel(wxWindow* parent);
    ~TabSingleCamPanel();

private:
    // --- UI Elements ---
    wxTextCtrl* textCtrlFilePath_;
    wxButton* btnSelectFolder_;

    wxTextCtrl* textCtrlRows_; // Corresponds to Python's m_textCtrl_row (Number of Rows of *squares*)
    wxTextCtrl* textCtrlCols_; // Corresponds to Python's m_textCtrl_col (Number of Cols of *squares*)
    wxTextCtrl* textCtrlCellSize_;
    wxCheckBox* checkBoxUseCustomDetector_; // Corresponds to use_libcbdetect

    wxButton* btnCalibrate_;
    wxButton* btnSaveResults_;
    wxButton* btnShowDistribution_;
    wxStaticText* staticTextWarning_;

    wxTreeCtrl* treeCtrlImages_;
    ImagePanel* imagePanelMainView_;
    // VTKPanel* cameraPoseView_; // VTKPanel not yet translated

    wxImageList* iconList_; // For tree control icons

    // --- Event Handlers ---
    void OnSelectFolder(wxCommandEvent& event);
    void OnCalibrate(wxCommandEvent& event);
    void OnSaveResults(wxCommandEvent& event);
    void OnShowDistribution(wxCommandEvent& event);
    void OnTreeItemSelect(wxTreeEvent& event);
    void OnTreeItemRightClick(wxTreeEvent& event);
    void OnRecalibrateFromMenu(wxCommandEvent& event);
    void OnTextChanged(wxCommandEvent& event);
    void OnCalibrationDone(CalibrationDoneEvent& event);

    // --- Helper Methods ---
    void LayoutControls();
    void UpdateButtonsState();
    void PopulateImageTree();
    void DisplayImageAndCorners(const std::string& filename);
    std::vector<std::filesystem::path> GetImagePathsFromStore(bool include_rejected = false);
    void UpdateTreeWithCalibrationStatus(bool highlight_max_error = true);
    void ClearImagePanel();


    // --- State Variables ---
    std::filesystem::path currentImageDir_;
    std::unique_ptr<core::calib::CalibBoard> calib_board_;
    core::calib::MonoCalibResult last_calibration_result_;

    wxString tree_item_path_for_menu_;

    struct ImageCalibData {
        std::filesystem::path file_path; // Full path for easy access
        bool rejected = false;
        double reprojection_error = -1.0; // Per-image reprojection error
        std::vector<cv::Point2f> corners;
        bool corners_found = false;
        // cv::Mat rvec, tvec; // If storing per-image pose
    };
    // Using a map for easier access by filename, though Python used an in-memory SQL DB
    std::map<std::string, ImageCalibData> image_data_store_;


    // Thread for calibration
    std::thread calibration_thread_;
    wxProgressDialog* progress_dialog_ = nullptr;

    // Event IDs for menu items and controls
    enum ControlIds {
        ID_SELECT_FOLDER = wxID_HIGHEST + 1,
        ID_ROWS_TEXT,
        ID_COLS_TEXT,
        ID_CELLSIZE_TEXT,
        ID_CALIBRATE_BTN,
        ID_SAVE_BTN,
        ID_SHOW_DIST_BTN,
        ID_TREE_IMAGES,
        ID_RECALIBRATE_MENU_ITEM
    };
};

#endif // TAB_SINGLE_CAM_H
