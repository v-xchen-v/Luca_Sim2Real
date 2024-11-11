class ICPFitnessException(Exception):
    """
    Custom exception for icp-related errors.

    Attributes:
    - message: Explanation of the error.
    """
    def __init__(self, message="Icp fitness error exceeds threshold."):
        self.message = message
        super().__init__(self.message)