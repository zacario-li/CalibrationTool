#ifndef STEREO_FILE_LOADER_DIALOG_H
#define STEREO_FILE_LOADER_DIALOG_H

#include <wx/wx.h>
#include <wx/dialog.h>
#include <wx/textctrl.h>
#include <wx/button.h>
#include <wx/stattext.h>
#include <string>

struct StereoFileLoaderData {
    std::string left_folder_path;
    std::string right_folder_path;
    long board_rows_squares = 9; // Number of SQUARES (e.g., vertical)
    long board_cols_squares = 12; // Number of SQUARES (e.g., horizontal)
    double cell_size_mm = 5.0;
    bool use_custom_detector = false; // Added, though not in Python dialog, useful for TabStereoCam
};

class StereoFileLoaderDialog : public wxDialog {
public:
    StereoFileLoaderDialog(wxWindow* parent, const wxString& title, const StereoFileLoaderData& initial_data);

    const StereoFileLoaderData& GetData() const { return data_; }

private:
    void OnSelectFolder(wxCommandEvent& event);
    void OnOK(wxCommandEvent& event);
    void OnTextChanged(wxCommandEvent& event);
    void UpdateOKButtonState();

    StereoFileLoaderData data_;

    // UI Elements
    wxTextCtrl* textCtrlLeftPath_;
    wxTextCtrl* textCtrlRightPath_;
    wxButton* btnSelectLeft_;
    wxButton* btnSelectRight_;

    wxTextCtrl* textCtrlBoardRows_;    // Num squares for rows
    wxTextCtrl* textCtrlBoardCols_;    // Num squares for columns
    wxTextCtrl* textCtrlCellSize_;
    // wxCheckBox* checkBoxUseCustomDetector_; // Could add this if needed from dialog

    wxButton* okButton_;

    enum ControlIds {
        ID_SELECT_LEFT_FOLDER = wxID_HIGHEST + 100,
        ID_SELECT_RIGHT_FOLDER,
        ID_BOARD_ROWS_TEXT,
        ID_BOARD_COLS_TEXT,
        ID_CELL_SIZE_TEXT
    };
};

#endif // STEREO_FILE_LOADER_DIALOG_H
