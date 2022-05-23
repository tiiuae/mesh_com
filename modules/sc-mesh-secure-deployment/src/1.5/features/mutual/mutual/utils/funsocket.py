import socket
from .wifi_ssrc import get_ip_address


def client_auth(ID, ser_ip, message):
    '''
    Socket client to send message to specific server.
    '''

    HOST = ser_ip  # The server's hostname or IP address
    PORT = int(ID.split('node')[1])  # Port to listen on (non-privileged ports are > 1023)
    print(f'Starting client Auth with {str(HOST)}:{PORT}')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        s.sendall(message)
        data = s.recv(1024)
    print('Sent: ', repr(data))


def server_auth(ID, interface='wlan0'):
    '''
    Create a socket server and get the key information to import it.
    '''
    ip = get_ip_address(interface)  # assuming that wlan0 will be (or connected to) the 'AP'
    HOST = ip
    PORT = int(ID.split('node')[1])  # Port to listen on (non-privileged ports are > 1023)
    print(f'Starting server Auth on {str(HOST)}:{PORT}')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        with conn:
            print("Connected by", addr)
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                conn.sendall(data)
                s.close()
                print('Received: ', repr(data))
                return data, addr