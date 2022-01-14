import gnupg
import os
from os import path
import glob
import wifi_ssrc as wf
from mesh import *
import socket
import time


def folder():
    if not path.isdir(PATH):
        os.mkdir(PATH)


# always install it with python3 gnupg, do not use pip3 install gnupg (different libraries)
PATH = 'auth/'
folder()
try:
    gpg = gnupg.GPG(gnupghome='.')
except TypeError:
    gpg = gnupg.GPG(homedir='.')
os.environ['GNUPGHOME'] = 'auth/'


# def __init__():
def init():
    '''
    Import the keys: node Pub,Priv and server pub.
    Returns ID of the node: obtained from the certificate.
    '''
    files = glob.glob('hsm/*.asc')
    for fi in files:
        aux = open(fi, 'rb')  # assuming it's the first and only certificate
        fi2 = aux.read()
        gpg.import_keys(fi2)
    for ky in gpg.list_keys():
        if ky["uids"] == ["provServer <provServer>"]:
            fp = ky["fingerprint"]
    gpg.trust_keys(fp, 'TRUST_FULLY')  #trusting the server key
    for i in gpg.list_keys():
        if 'node' in i['uids'][0]:
            ID = i['uids'][0].split(' <')[0]
    return ID


def decrypt_conf(ID):
    '''
    Decrypt the mesh_conf file
    '''
    with open("hsm/" + ID + "_conf.conf.gpg", "rb") as f:
        status = gpg.decrypt_file(f, output="src/mesh_conf.conf")  # status has some info from the signer
        if status.ok:
            print("Mesh_conf file decrypt successful ")
        else:
            print("Mesh_conf file decrypt successful error")
            print("status: ", status.status)


def server_auth(ID, interface='wlan0'):
    ip = wf.get_ip_address(interface)  # assuming that wlan0 will be (or connected to) the 'AP'
    HOST = ip
    PORT = ID.split('node')[1]  # Port to listen on (non-privileged ports are > 1023)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
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
                return data


def authentication(ID):
    ctry = 0
    client_key = server_auth(ID)
    ck = gpg.import_keys(client_key)
    fpr = ck.fingerprint
    while ctry <= 3:
        if not fpr:  # checking if list is empty
            client_key = server_auth(ID)
            ctry = + 1
        else:
            print('not able to receive client key')
            break
    for ky in gpg.list_keys(sigs=True):  # checking if key was signed by Server
        if ky["fingerprint"] == fpr[0]:
            for sig in ky["sigs"]:
                if ("provServer" in sig[1]  # signed
                        and float(ky['expire']) > time.time()):  # not expired
                    level = 3

    #todo needs to verify if has been signed by an already trusted node
    #needs to set what happened if not signed
    #if level > 3, exchange public key, define password if not set. received ack



if __name__ == "__main__":
    ID = init()
    candidate = wf.scan_wifi()  # scan wifi to authenticate with
    if candidate:
        wf.connect_wifi(candidate)
        authenticate(ID)
        create_mesh(ID)
    else:  # no wifi available need to start mesh by itself
        wf.create_ap(ID)
        decrypt_conf(ID)
        password = create_password()
        update_password(password)
        create_mesh(ID)

'''
import gnupg
import os
cs_gpg_options = ["--pinentry-mode loopback"]
try:
    gpg = gnupg.GPG(gnupghome=".", options=cs_gpg_options)
except TypeError:
    gpg = gnupg.GPG(homedir=".", options=cs_gpg_options)
os.environ["GNUPGHOME"] = "auth/"
import glob
files = glob.glob('hsm/*.asc')
for fi in files:
    aux = open(fi, 'rb')  # assuming it's the first and only certificate
    fi = aux.read()
    gpg.import_keys(fi)
gpg.list_keys()
'''
