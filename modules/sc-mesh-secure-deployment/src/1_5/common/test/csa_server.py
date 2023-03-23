import sys
sys.path.append('../')
from csa import Csa_Server

if __name__ == '__main__':
    server = Csa_Server('127.0.0.1', 54321)
    conn = server.accept()
    server.send('CSAACK:SA:b3f3:seq:1'.encode())
    data = server.recv().decode()
    print(data)
    server.close()
