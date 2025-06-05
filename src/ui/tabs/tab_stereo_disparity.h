#ifndef TAB_STEREO_DISPARITY_H
#define TAB_STEREO_DISPARITY_H

#include <wx/wx.h>
#include <wx/panel.h>
#include <wx/textctrl.h>
#include <wx/button.h>
#include <wx/radiobox.h>
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
#include <ui/components/custom_events.h> // For custom events if needed for async results
#include <ui/dialogs/stereo_file_loader_dialog.h> // For StereoFileLoaderData (paths only)
#include <core/depth_processing/sgbm_processor.h>
#include <core/common/file_utils.h> // For load_camera_param_from_json

// VTK Forward Declarations
class vtkPoints;
class vtkUnsignedCharArray;
class vtkRenderer;
class vtkRenderWindow;
class vtkRenderWindowInteractor;
class vtkPolyData;
class vtkActor;

// Custom event for Disparity/Depth computation completion
class StereoProcessingDoneEvent;
wxDECLARE_EVENT(EVT_STEREO_PROCESSING_DONE, StereoProcessingDoneEvent);

enum class StereoProcessingType {
    DISPARITY,
    DEPTH_AND_VISUALIZE
};

class StereoProcessingDoneEvent : public wxCommandEvent {
public:
    StereoProcessingDoneEvent(wxEventType eventType = EVT_STEREO_PROCESSING_DONE, int id = 0)
        : wxCommandEvent(eventType, id), type_(StereoProcessingType::DISPARITY) {}
    StereoProcessingDoneEvent(const StereoProcessingDoneEvent& event)
        : wxCommandEvent(event),
          type_(event.type_),
          disparity_map_(event.disparity_map_), // cv::Mat uses shallow copy by default
          points_data_(event.points_data_),     // Smart pointers would be better for vtk objects
          colors_data_(event.colors_data_) {}

    virtual wxEvent* Clone() const override { return new StereoProcessingDoneEvent(*this); }

    void SetProcessingType(StereoProcessingType type) { type_ = type; }
    StereoProcessingType GetProcessingType() const { return type_; }

    void SetDisparityMap(const cv::Mat& disp) { disparity_map_ = disp.clone(); } // Clone for safety
    cv::Mat GetDisparityMap() const { return disparity_map_; }

    // For VTK data, passing raw pointers is risky.
    // For simplicity here, but shared_ptr or other ownership models are better.
    // Or, better, process VTK fully in worker and only signal completion.
    // For this example, we'll pass basic data, and VTK objects created in UI thread.
    // Let's simplify: the event will just signal completion, data retrieved from member variables.

private:
    StereoProcessingType type_;
    cv::Mat disparity_map_; // For DISPARITY type
    // For DEPTH_AND_VISUALIZE, the main thread will access member variables
    // updated by the worker thread (e.g., point cloud data file path).
    // Or, pass minimal data needed for VTK setup if VTK objects are too complex for events.
    vtkSmartPointer<vtkPoints> points_data_; // Example if passing VTK data (needs vtkSmartPointer.h)
    vtkSmartPointer<vtkUnsignedCharArray> colors_data_;
};

typedef void (wxEvtHandler::*StereoProcessingDoneEventFunction)(StereoProcessingDoneEvent&);
#define EVT_STEREO_PROCESSING_DONE_HANDLER(id, fn) \
    wx__DECLARE_EVT1(EVT_STEREO_PROCESSING_DONE, id, StereoProcessingDoneEventFunction(fn))


class TabStereoDisparityPanel : public wxPanel {
public:
    TabStereoDisparityPanel(wxWindow* parent);
    ~TabStereoDisparityPanel();

private:
    // --- UI Elements ---
    // SGBM Parameters
    wxRadioBox* radioBoxSgbmMode_;
    wxTextCtrl* textCtrlSgbmBlockSize_;
    wxTextCtrl* textCtrlSgbmP1_, *textCtrlSgbmP2_;
    wxTextCtrl* textCtrlSgbmMinDisparity_, *textCtrlSgbmNumDisparities_;
    wxTextCtrl* textCtrlSgbmDisp12MaxDiff_, *textCtrlSgbmPreFilterCap_;
    wxTextCtrl* textCtrlSgbmUniquenessRatio_, *textCtrlSgbmSpeckleWinSize_, *textCtrlSgbmSpeckleRange_;
    wxTextCtrl* textCtrlZLimitMm_; // For depth filtering

    // Camera Parameters & Image Loading
    wxTextCtrl* textCtrlCameraParamsPath_;
    wxButton* btnLoadCameraParams_;
    wxButton* btnLoadImages_; // Uses StereoFileLoaderDialog or similar logic

    // Operations
    wxButton* btnComputeDisparity_;
    wxButton* btnRectifyImages_; // Shows rectified images with lines
    wxButton* btnComputeDepth_;    // Computes depth and shows point cloud
    wxButton* btnSaveResults_;

    // Display
    wxTreeCtrl* treeCtrlImages_;
    ImagePanel* imagePanelLeftRawOrRect_; // Displays left raw or rectified image
    ImagePanel* imagePanelDisparity_;     // Displays disparity map
    wxImageList* iconList_;

    // VTK display related (managed by ShowPointCloudVTK)
    vtkSmartPointer<vtkRenderer> vtk_renderer_;
    vtkSmartPointer<vtkRenderWindow> vtk_render_window_;
    vtkSmartPointer<vtkRenderWindowInteractor> vtk_interactor_;


    // --- Event Handlers ---
    void OnLoadCameraParams(wxCommandEvent& event);
    void OnLoadImages(wxCommandEvent& event);
    void OnSgbmParamChanged(wxCommandEvent& event); // For any SGBM param TextCtrl or RadioBox
    void OnComputeDisparity(wxCommandEvent& event);
    void OnRectifyImages(wxCommandEvent& event);
    void OnComputeDepth(wxCommandEvent& event);
    void OnSaveResults(wxCommandEvent& event);
    void OnTreeItemSelect(wxTreeEvent& event);
    void OnStereoProcessingDone(StereoProcessingDoneEvent& event);

    // --- Helper Methods ---
    void LayoutControls();
    void UpdateSgbmProcessorFromUI(); // Reads UI params and updates/initializes SgbmProcessor
    void PopulateImageTree();
    void UpdateButtonStates();
    void ClearImagePanels();
    void ShowPointCloudVTK(vtkSmartPointer<vtkPoints> points, vtkSmartPointer<vtkUnsignedCharArray> colors);


    // --- State Variables ---
    std::unique_ptr<core::depth::SgbmProcessor> sgbm_processor_;
    core::depth::SgbmParams current_sgbm_params_; // Stores current UI SGBM settings

    std::string camera_params_file_path_;
    std::string left_image_folder_path_;
    std::string right_image_folder_path_;
    cv::Size input_image_size_ = {0,0}; // Detected from first loaded image pair

    struct DisparityImageData {
        std::filesystem::path left_image_path;
        std::filesystem::path right_image_path;
        cv::Mat disparity_map; // CV_16S usually
        cv::Mat left_rectified_img; // For display and point cloud coloring
        cv::Mat right_rectified_img; // For display
        // Store path to PCD file instead of raw points to avoid large memory in map
        std::string pcd_file_path;
        bool is_processed = false; // Disparity computed
    };
    // Map key: left image filename
    std::map<std::string, DisparityImageData> image_data_store_;
    std::string selected_left_image_key_; // Filename of selected left image in tree


    // Threading
    std::thread processing_thread_;
    wxProgressDialog* progress_dialog_ = nullptr;

    enum ControlIds {
        // Define IDs for controls that need specific event handling or access
        ID_LOAD_CAM_PARAMS = wxID_HIGHEST + 400,
        ID_LOAD_IMAGES,
        ID_COMPUTE_DISPARITY,
        ID_RECTIFY_IMAGES,
        ID_COMPUTE_DEPTH,
        ID_SAVE_RESULTS,
        ID_TREE_IMAGES,
        // SGBM Param IDs (can use wxID_ANY if generic handler is smart enough)
        ID_SGBM_MODE, ID_SGBM_BLOCKSIZE, ID_SGBM_P1, ID_SGBM_P2,
        ID_SGBM_MINDISP, ID_SGBM_NUMDISP, ID_SGBM_DISP12MAXDIFF,
        ID_SGBM_PREFILTERCAP, ID_SGBM_UNIQUENESS, ID_SGBM_SPECKLEWIN, ID_SGBM_SPECKLERANGE,
        ID_SGBM_ZLIMIT
    };
};

#endif // TAB_STEREO_DISPARITY_H
