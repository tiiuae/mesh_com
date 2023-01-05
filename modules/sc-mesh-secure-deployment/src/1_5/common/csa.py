import socket

class Csa_Server:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((host, port))
        self.socket.listen(1)

    def accept(self):
        self.conn, self.addr = self.socket.accept()
        print('Connected by ', self.addr)
        return self.conn

    def recv(self):
        msg = self.conn.recv(1024)
        return msg

    def send(self, msg):
        self.conn.send(msg)

    def close(self):
        self.conn.close()

class Csa_Client:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect(self):
        self.socket.connect((self.host, self.port))

    def send(self, msg):
        self.socket.send(msg)

    def recv(self):
        msg = self.socket.recv(1024)
        return msg

    def close(self):
        self.socket.close()

