#ifndef CUSTOM_EVENTS_H
#define CUSTOM_EVENTS_H

#include <wx/event.h>
#include <wx/string.h>
#include <opencv2/core/mat.hpp> // For potential cv::Mat payload, though not used in this example

// Define a new event type for generic string messages
wxDECLARE_EVENT(EVT_CUSTOM_STRING_MESSAGE, wxCommandEvent);

// Define a new event type for image updates (example, could carry path or cv::Mat)
// For simplicity, this example won't carry a cv::Mat directly in the event object
// to avoid complexities with ownership and large data in events.
// Instead, it could carry an ID or a signal to refresh from a known source.
// If cv::Mat is needed, consider wxImage or a shared pointer.
wxDECLARE_EVENT(EVT_IMAGE_UPDATE_VISUAL, wxCommandEvent);


// Generic data-carrying event (similar to Python's CustomEvent)
// This is more complex in C++ due to type safety and ownership.
// Using wxString as an example payload.
class DataEvent : public wxCommandEvent {
public:
    DataEvent(wxEventType eventType = wxEVT_NULL, int id = 0)
        : wxCommandEvent(eventType, id) {}

    // Clone constructor
    DataEvent(const DataEvent& event)
        : wxCommandEvent(event), data_string_(event.data_string_) {}

    virtual wxEvent* Clone() const override {
        return new DataEvent(*this);
    }

    void SetDataString(const wxString& str) { data_string_ = str; }
    wxString GetDataString() const { return data_string_; }

    // If you needed to carry a cv::Mat, it would be more complex:
    // cv::Mat mat_payload;
    // void SetMatPayload(const cv::Mat& mat) { mat_payload = mat.clone(); } // Clone to manage lifetime
    // cv::Mat GetMatPayload() const { return mat_payload; }


private:
    wxString data_string_;
    // cv::Mat mat_payload_; // Example if carrying cv::Mat
};

// Define the event type for DataEvent
wxDECLARE_EVENT(EVT_DATA_EVENT, DataEvent);

// Typedef for the event handler function for DataEvent
typedef void (wxEvtHandler::*DataEventFunction)(DataEvent&);

// Macro for the event table
#define EVT_DATA(id, fn) \
    wx__DECLARE_EVT1(EVT_DATA_EVENT, id, DataEventFunction(fn))


#endif // CUSTOM_EVENTS_H
