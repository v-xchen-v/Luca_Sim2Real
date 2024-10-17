class CalibrationBoardNotFoundError(Exception):
    """
    Custom exception for calibration-related errors.

    Attributes:
    - message: Explanation of the error.
    """
    def __init__(self, message="Calibration board not found."):
        self.message = message
        super().__init__(self.message)
        
class ReprojectionThresholdExceededError(Exception):
    """
    Custom exception for calibration-related errors.

    Attributes:
    - message: Explanation of the error.
    """
    def __init__(self, message="Reprojection error exceeds threshold."):
        self.message = message
        super().__init__(self.message)