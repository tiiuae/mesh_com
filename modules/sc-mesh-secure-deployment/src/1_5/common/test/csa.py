import socket

class Csa_Server:
    def __init__(self, interface_name, port):
        self.host = None # Listen on all available addresses
        self.port = port
        self.socket = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Resolve the IP address of the given interface name
        addrinfo = socket.getaddrinfo(interface_name, None, socket.AF_INET6, socket.SOCK_STREAM)
        self.ip_address = addrinfo[0][4][0]

        self.socket.bind((self.ip_address, port))
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
        self.socket = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)

    def connect(self):
        addrinfo = socket.getaddrinfo(self.host, self.port, socket.AF_INET6, socket.SOCK_STREAM)
        self.socket.connect(addrinfo[0][4])

    def send(self, msg):
        self.socket.send(msg)

    def recv(self):
        msg = self.socket.recv(1024)
        return msg

    def close(self):
        self.socket.close()
