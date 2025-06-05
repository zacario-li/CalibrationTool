#ifndef APP_ERRORS_H
#define APP_ERRORS_H

#include <string>
#include <stdexcept> // For std::out_of_range

namespace core {
namespace common {

enum class CalibErrType {
    CAL_OK,
    CAL_CORNER_DET_ERR,
    CAL_DATA_SIZE_NOT_MATCH,
    CAL_DATA_CSV_FORMAT_ERR
};

inline std::string to_string(CalibErrType err_type) {
    switch (err_type) {
        case CalibErrType::CAL_OK:
            return "CAL_OK";
        case CalibErrType::CAL_CORNER_DET_ERR:
            return "CAL_CORNER_DET_ERR";
        case CalibErrType::CAL_DATA_SIZE_NOT_MATCH:
            return "CAL_DATA_SIZE_NOT_MATCH";
        case CalibErrType::CAL_DATA_CSV_FORMAT_ERR:
            return "CAL_DATA_CSV_FORMAT_ERR";
        default:
            // This case should ideally not be reached if all enum values are handled.
            // Throwing an exception or returning a specific "Unknown error" string
            // can be ways to handle unexpected values.
            return "Unknown CalibErrType";
    }
}

} // namespace common
} // namespace core

#endif // APP_ERRORS_H
