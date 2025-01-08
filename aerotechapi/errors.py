class ConnectionError(Exception):

    def __init__(self, message):
        super().__init__(message)


class CommandFaultError(Exception):
    '''
    Exception is raised when the controller returns a command fault error
    flag.
    '''

    def __init__(self, message):
        super().__init__(message)


class CommandInvalidError(Exception):
    '''
    Exception is raised when the controller returns a command invalid error
    flag.
    '''

    def __init__(self, message):
        super().__init__(message)