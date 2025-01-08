from .constants import COMMAND_SUCCESS_CHAR, COMMAND_FAULT_CHAR, COMMAND_INVALID_CHAR, EOS_CHAR

class Response:

    command_parameters = {COMMAND_SUCCESS_CHAR: 'CommandSuccess',
                          COMMAND_INVALID_CHAR: 'CommandInvalid',
                          COMMAND_FAULT_CHAR: 'CommandFault'}

    def __init__(self, msg_str):
        self.data, self.status = self.deconstruct_msg_str(msg_str)

    def __repr__(self):
        if self.data:
            return f'{self.status}, {self.data}'
        else:
            return f'{self.status}'

    def __str__(self):
        if self.data:
            return self.data
        else:
            return self.status

    def deconstruct_msg_str(self, msg_str):
        if msg_str.endswith(EOS_CHAR):
            msg_str = msg_str.rstrip(EOS_CHAR)
        data = msg_str[1:]
        command_parameter = self.command_parameters[msg_str[0]]
        return data, command_parameter