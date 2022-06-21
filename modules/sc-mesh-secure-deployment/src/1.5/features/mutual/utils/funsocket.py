import socket
import sys
sys.path.insert(0, '../../')
import random
'''
only for testing 
'''
from common import ConnectionMgr
co = ConnectionMgr.ConnectionMgr()


def client_auth(ID, ser_ip, message, interface='wlan0'):
    '''
    Socket client to send message to specific server.
    '''
    HOST = ser_ip  # The server's hostname or IP address
    PORT = int(ID.split('AuthAP_')[1]) if 'AuthAP_' in ID else int(ID)
    print(f'Starting client Auth with {str(HOST)}:{PORT}')
    ipaddr = co.get_ip_address(interface)  # assuming that wlan0 will be (or connected to) the 'AP'
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((ipaddr, random.randint(1000, 64000)))
        s.connect((HOST, PORT))
        s.sendall(message)
        data = s.recv(2048)
    print('Sent: ', repr(data))


def server_auth(ID, interface='wlan0'):
    '''
    Create a socket server and get the key information to import it.
    '''
    ip = co.get_ip_address(interface)  # assuming that wlan0 will be (or connected to) the 'AP'
    HOST = ip
    PORT = int(ID.split('AuthAP_')[1]) if 'AuthAP_' in ID else int(ID)
    print(f'Starting server Auth on {str(HOST)}:{PORT}')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        with conn:
            print("Connected by", addr)
            while True:
                data = conn.recv(2048)
                if not data:
                    break
                conn.sendall(data)
                s.close()
                print('Received: ', repr(data))
                return data, addr
