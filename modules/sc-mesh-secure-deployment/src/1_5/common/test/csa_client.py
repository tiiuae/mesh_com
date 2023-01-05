import sys
sys.path.append('../')
from csa import Csa_Client

if __name__ == '__main__':
    client = Csa_Client('127.0.0.1', 54321)
    client.connect()
    data = client.recv().decode()
    print(data)
    client.send('CSA:SA:b3f3:ch:6:seq:1'.encode())
    client.close()
