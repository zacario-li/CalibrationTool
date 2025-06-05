#include "stereo_file_loader_dialog.h"
#include <wx/dirdlg.h>
#include <wx/valnum.h> // For wxFloatingPointValidator, wxIntegerValidator
#include <wx/sizer.h>
#include <filesystem> // For path validation (optional, wxDirDialog handles existence)

namespace fs = std::filesystem;

StereoFileLoaderDialog::StereoFileLoaderDialog(wxWindow* parent, const wxString& title, const StereoFileLoaderData& initial_data)
    : wxDialog(parent, wxID_ANY, title, wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER),
      data_(initial_data) {

    wxBoxSizer* mainSizer = new wxBoxSizer(wxVERTICAL);

    // --- Path Selection ---
    wxStaticBoxSizer* pathSizer = new wxStaticBoxSizer(wxVERTICAL, this, "Image Folders");

    wxBoxSizer* leftPathHBox = new wxBoxSizer(wxHORIZONTAL);
    leftPathHBox->Add(new wxStaticText(pathSizer->GetStaticBox(), wxID_ANY, "Left Cam Folder:"), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    textCtrlLeftPath_ = new wxTextCtrl(pathSizer->GetStaticBox(), wxID_ANY, data_.left_folder_path, wxDefaultPosition, wxSize(300, -1), wxTE_READONLY);
    btnSelectLeft_ = new wxButton(pathSizer->GetStaticBox(), ID_SELECT_LEFT_FOLDER, "...");
    leftPathHBox->Add(textCtrlLeftPath_, 1, wxEXPAND | wxRIGHT, 5);
    leftPathHBox->Add(btnSelectLeft_, 0, wxALIGN_CENTER_VERTICAL, 0);
    pathSizer->Add(leftPathHBox, 0, wxEXPAND | wxALL, 5);

    wxBoxSizer* rightPathHBox = new wxBoxSizer(wxHORIZONTAL);
    rightPathHBox->Add(new wxStaticText(pathSizer->GetStaticBox(), wxID_ANY, "Right Cam Folder:"), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 5);
    textCtrlRightPath_ = new wxTextCtrl(pathSizer->GetStaticBox(), wxID_ANY, data_.right_folder_path, wxDefaultPosition, wxSize(300, -1), wxTE_READONLY);
    btnSelectRight_ = new wxButton(pathSizer->GetStaticBox(), ID_SELECT_RIGHT_FOLDER, "...");
    rightPathHBox->Add(textCtrlRightPath_, 1, wxEXPAND | wxRIGHT, 5);
    rightPathHBox->Add(btnSelectRight_, 0, wxALIGN_CENTER_VERTICAL, 0);
    pathSizer->Add(rightPathHBox, 0, wxEXPAND | wxALL, 5);
    mainSizer->Add(pathSizer, 0, wxEXPAND | wxALL, 5);

    // --- Board Parameters ---
    wxStaticBoxSizer* boardParamsSizer = new wxStaticBoxSizer(wxVERTICAL, this, "Checkerboard Parameters");
    wxFlexGridSizer* flexGridSizer = new wxFlexGridSizer(3, 2, 5, 5); // 3 rows, 2 cols, 5px gaps
    flexGridSizer->AddGrowableCol(1);

    flexGridSizer->Add(new wxStaticText(boardParamsSizer->GetStaticBox(), wxID_ANY, "Board Rows (squares):"), 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT);
    textCtrlBoardRows_ = new wxTextCtrl(boardParamsSizer->GetStaticBox(), ID_BOARD_ROWS_TEXT, wxString::Format("%ld", data_.board_rows_squares));
    textCtrlBoardRows_->SetValidator(wxIntegerValidator<long>(&data_.board_rows_squares, 2, 100)); // Min 2 squares
    flexGridSizer->Add(textCtrlBoardRows_, 1, wxEXPAND);

    flexGridSizer->Add(new wxStaticText(boardParamsSizer->GetStaticBox(), wxID_ANY, "Board Cols (squares):"), 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT);
    textCtrlBoardCols_ = new wxTextCtrl(boardParamsSizer->GetStaticBox(), ID_BOARD_COLS_TEXT, wxString::Format("%ld", data_.board_cols_squares));
    textCtrlBoardCols_->SetValidator(wxIntegerValidator<long>(&data_.board_cols_squares, 2, 100));
    flexGridSizer->Add(textCtrlBoardCols_, 1, wxEXPAND);

    flexGridSizer->Add(new wxStaticText(boardParamsSizer->GetStaticBox(), wxID_ANY, "Cell Size (mm):"), 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT);
    textCtrlCellSize_ = new wxTextCtrl(boardParamsSizer->GetStaticBox(), ID_CELL_SIZE_TEXT, wxString::Format("%.2f", data_.cell_size_mm));
    textCtrlCellSize_->SetValidator(wxFloatingPointValidator<double>(&data_.cell_size_mm, 0.01, 1000.0, wxNUM_VAL_NO_TRAILING_ZEROES, 2));
    flexGridSizer->Add(textCtrlCellSize_, 1, wxEXPAND);
    boardParamsSizer->Add(flexGridSizer, 1, wxEXPAND | wxALL, 5);
    mainSizer->Add(boardParamsSizer, 0, wxEXPAND | wxALL, 5);

    // --- Dialog Buttons ---
    wxStdDialogButtonSizer* dialogButtons = new wxStdDialogButtonSizer();
    okButton_ = new wxButton(this, wxID_OK, "OK");
    dialogButtons->AddButton(okButton_);
    dialogButtons->AddButton(new wxButton(this, wxID_CANCEL, "Cancel"));
    dialogButtons->Realize();
    mainSizer->Add(dialogButtons, 0, wxALIGN_CENTER | wxALL, 10);

    SetSizerAndFit(mainSizer);
    CentreOnParent();

    // Bind events
    Bind(wxEVT_BUTTON, &StereoFileLoaderDialog::OnSelectFolder, this, ID_SELECT_LEFT_FOLDER);
    Bind(wxEVT_BUTTON, &StereoFileLoaderDialog::OnSelectFolder, this, ID_SELECT_RIGHT_FOLDER);
    Bind(wxEVT_BUTTON, &StereoFileLoaderDialog::OnOK, this, wxID_OK);

    // Bind text change events to update OK button state
    Bind(wxEVT_TEXT, &StereoFileLoaderDialog::OnTextChanged, this, ID_BOARD_ROWS_TEXT);
    Bind(wxEVT_TEXT, &StereoFileLoaderDialog::OnTextChanged, this, ID_BOARD_COLS_TEXT);
    Bind(wxEVT_TEXT, &StereoFileLoaderDialog::OnTextChanged, this, ID_CELL_SIZE_TEXT);
    // Also for paths, though they are read-only, they change via button
    Bind(wxEVT_TEXT, &StereoFileLoaderDialog::OnTextChanged, this, wxID_ANY); // Catch all text changes including paths

    UpdateOKButtonState(); // Initial state
}

void StereoFileLoaderDialog::OnSelectFolder(wxCommandEvent& event) {
    wxString current_path = (event.GetId() == ID_SELECT_LEFT_FOLDER) ? textCtrlLeftPath_->GetValue() : textCtrlRightPath_->GetValue();
    wxDirDialog dirDialog(this, "Select Image Folder", current_path, wxDD_DEFAULT_STYLE | wxDD_DIR_MUST_EXIST);

    if (dirDialog.ShowModal() == wxID_OK) {
        if (event.GetId() == ID_SELECT_LEFT_FOLDER) {
            textCtrlLeftPath_->SetValue(dirDialog.GetPath());
            data_.left_folder_path = dirDialog.GetPath().ToStdString();
        } else if (event.GetId() == ID_SELECT_RIGHT_FOLDER) {
            textCtrlRightPath_->SetValue(dirDialog.GetPath());
            data_.right_folder_path = dirDialog.GetPath().ToStdString();
        }
    }
    UpdateOKButtonState();
}

void StereoFileLoaderDialog::OnOK(wxCommandEvent& event) {
    if (Validate() && TransferDataFromWindow()) {
        // Additional validation if needed (e.g., paths are not empty, parameters are reasonable)
        if (data_.left_folder_path.empty() || data_.right_folder_path.empty()) {
            wxMessageBox("Both left and right image folders must be selected.", "Error", wxOK | wxICON_ERROR, this);
            return;
        }
        if (data_.board_rows_squares <= 1 || data_.board_cols_squares <= 1 || data_.cell_size_mm <= 0) {
             wxMessageBox("Board dimensions and cell size must be positive (squares > 1).", "Error", wxOK | wxICON_ERROR, this);
            return;
        }
        EndModal(wxID_OK);
    } else {
        // Validation failed, message box usually shown by wxValidators
    }
}

void StereoFileLoaderDialog::OnTextChanged(wxCommandEvent& event) {
    UpdateOKButtonState();
    event.Skip(); // Allow other handlers if any
}

void StereoFileLoaderDialog::UpdateOKButtonState() {
    bool left_path_ok = !textCtrlLeftPath_->GetValue().IsEmpty();
    bool right_path_ok = !textCtrlRightPath_->GetValue().IsEmpty();

    long r, c;
    double cs;
    bool rows_ok = textCtrlBoardRows_->GetValue().ToLong(&r) && r > 1;
    bool cols_ok = textCtrlBoardCols_->GetValue().ToLong(&c) && c > 1;
    bool cell_size_ok = textCtrlCellSize_->GetValue().ToDouble(&cs) && cs > 0;

    if (okButton_) { // okButton_ might not exist during early constructor calls if not careful
        okButton_->Enable(left_path_ok && right_path_ok && rows_ok && cols_ok && cell_size_ok);
    }
}
