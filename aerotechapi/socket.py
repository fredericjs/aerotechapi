import socket
from .reponse import Response
from .constants import EOS_CHAR

class _TcpIpSocketDummy:
    '''
    Tcp socket dummy Class that emulates the core functionality of a Tcp socket
    for testing purposes.
    '''

    def __init__(self):
        pass

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, tb):
        if exc_type is not None:
            traceback.print_exception(exc_type, exc_value, tb)
        self.disconnect()

    def send(self, message, blocking=True):
        print(message)
        return Response('%Test\n')

    def connect(self):
        print('Connected to controller.')
        pass

    def disconnect(self):
        print('Disconnected from controller.')
        pass


class _TcpIpSocket(socket.socket):
    '''
    Class that manages the TCP/IP connection to the socket running on the
    local host on port 8000.
    '''

    def __init__(self, port, address):
        super().__init__(socket.AF_INET, socket.SOCK_STREAM)
        self._server_address = address
        self._port = port
        self.settimeout(0.2)

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, tb):
        if exc_type is not None:
            traceback.print_exception(exc_type, exc_value, tb)
        self.disconnect()

    def connect(self):
        super().connect((self._server_address, self._port))

    def disconnect(self):
        self.close()

    def send(self, message):
        if not message.endswith(EOS_CHAR):
            message += EOS_CHAR
        super().send(message.encode())
        while True:
            ret = self._receive()
            if ret == None:
                continue
            else:
                break
        return ret[-1]

    def _receive(self):
        try:
            ret = self.recv(1024).decode()
            responses = [Response(msg) for msg in ret.split('\n') if msg != '']
            return responses
        except socket.timeout:
            return None