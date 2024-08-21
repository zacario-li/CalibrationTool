from enum import Enum, auto

class CalibErrType(Enum):
    CAL_OK = auto()
    CAL_CORNER_DET_ERR = auto()
    CAL_DATA_SIZE_NOT_MATCH = auto()
    CAL_DATA_CSV_FORMAT_ERR = auto()

    @staticmethod
    def to_string(err_type):
        return err_type.name if isinstance(err_type, CalibErrType) else "Unknown error"