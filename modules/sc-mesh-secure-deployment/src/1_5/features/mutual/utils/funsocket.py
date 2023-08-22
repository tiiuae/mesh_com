import socket
import sys

sys.path.insert(0, '../../')

from common import mesh_utils




def client_auth(ID, ser_ip, message, interface='wlan0'):
    '''
    Socket client to send message to specific server.
    '''
    HOST = ser_ip  # The server's hostname or IP address
    #PORT = int(ID.split('AuthAP_')[1]) if 'AuthAP_' in ID else int(ID)
    PORT = 7777
    print(f'Starting client Auth with {str(HOST)}:{PORT}')
    #ipaddr = co.get_ip_address(interface)  # assuming that wlan0 will be (or connected to) the 'AP'
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        #s.bind((ipaddr, random.randint(1000, 64000)))
        sock.connect((HOST, PORT))
        sock.sendall(message)
        data = sock.recv(2048)
    print('Sent: ', repr(data))


def server_auth(ID, interface='wlan0'):
    '''
    Create a socket server and get the key information to import it.
    '''
    ip =  mesh_utils.get_mesh_ip_address(interface)  # assuming that wlan0 will be (or connected to) the 'AP'
    HOST = ip
    #PORT = int(ID.split('AuthAP_')[1]) if 'AuthAP_' in ID else int(ID)
    PORT = 7777
    print(f'Starting server Auth on {str(HOST)}:{PORT}')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((HOST, PORT))
        sock.listen()

        conn, addr = sock.accept()
        with conn:
            print("Connected by", addr)
            while True:
                data = conn.recv(2048)
                if not data:
                    break
                conn.sendall(data)
                sock.close()
                print('Received: ', repr(data))
                return data, addr
