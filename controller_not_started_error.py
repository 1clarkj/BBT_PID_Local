class ControllerNotStartedError(Exception):
    def __init__(self):
        pass

    def __str__(self):
        return "Please start and close the communicator gracefully."
