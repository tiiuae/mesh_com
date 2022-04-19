from PyKCS11 import *
import subprocess
import hashlib

BUF_SIZE = 65536  # lets read stuff in 64kb chunks!
LIB = "/usr/lib/softhsm/libsofthsm2.so"
os.environ['PYKCS11LIB'] = LIB

debug = True

'''
TODO:create a function to verify if keys exist and are valid
'''


class Primitives:
    def __init__(self):
        pkcs11 = PyKCS11Lib()
        pkcs11.load()  # define environment variable PYKCS11LIB=YourPKCS11Lib

        # get 1st slot
        slot = pkcs11.getSlotList(tokenPresent=True)[0]

        self.session = pkcs11.openSession(slot, CKF_SERIAL_SESSION | CKF_RW_SESSION)
        self.session.login("1234")

        keys = self.session.findObjects()
        for key in range(len(keys)):
            self.session.destroyObject(keys[key])

    def __exit__(self):
        self.session.logout()
        self.session.closeSession()

    def encrypt_response(self, message,
                         name):  # assuming that data is on a file called payload.enc generated on the function get_data
        keys = self.session.findObjects()
        for key in range(len(keys)):
            aux = keys[key].to_dict()
            if aux['CKA_LABEL'] == name:
                pubKey = keys[key]

        encr = self.session.encrypt(pubKey, message)
        if debug:
            print(f'> Encrypted message: {bytes(encr)}')
        # logout
        return bytes(encr)

    def decrypt_response(self, encr):  # assuming that data is on a file called payload.enc generated on the function get_data
        privKey = self.session.findObjects([(CKA_CLASS, CKO_PRIVATE_KEY)])[0]
        dec = self.session.decrypt(privKey, encr)
        if type(dec) == 'PyKCS11.ckbytelist':
            dec = bytes(dec)
        if debug:
            print(f'> Decrypted message: {str(bytes(dec))}')
        # logout
        return dec

    def delete_key(self, node_name): #check if it is possible to delete on python (destroy_objetc)
        label = f'--label={node_name}'
        command = ['pkcs11-tool', '--module', LIB, '--delete-object', label, '--type=pubkey']
        subprocess.call(command, shell=False)

    def import_cert(self, client_key, node_name):
        LIB = '/usr/lib/softhsm/libsofthsm2.so'
        filename = f'{node_name}.der'
        id = node_name.split('_')[-1]
        with open(filename, 'wb') as writer:
            writer.write(client_key.read())
        command = ['pkcs11-tool', '--module', LIB, '-l', '--pin', '1234', '--write-object', filename, '--type',
                   'pubkey',
                   '--id', id, '--label', node_name]
        subprocess.call(command, shell=False)

    def verify_hsm(self, msg, sig, name):
        pkcs11 = PyKCS11Lib()
        pkcs11.load()  # define environment variable PYKCS11LIB=YourPKCS11Lib
        # get 1st slot
        slot = pkcs11.getSlotList(tokenPresent=True)[0]
        session = pkcs11.openSession(slot, CKF_SERIAL_SESSION | CKF_RW_SESSION)
        session.login("1234")
        keys = session.findObjects()
        for key in range(len(keys)):
            aux = keys[key].to_dict()
            if aux['CKA_LABEL'] == name:
                pubKey = keys[key]  ##check here need to verify if not exported
        ver = session.verify(pubKey, msg, sig)
        # logout
        session.logout()
        session.closeSession()
        return ver

    def sign_hsm(self, msg):
        pkcs11 = PyKCS11Lib()
        pkcs11.load()  # define environment variable PYKCS11LIB=YourPKCS11Lib
        # get 1st slot
        slot = pkcs11.getSlotList(tokenPresent=True)[0]
        session = pkcs11.openSession(slot, CKF_SERIAL_SESSION | CKF_RW_SESSION)
        session.login("1234")
        privKey = session.findObjects([(CKA_CLASS, CKO_PRIVATE_KEY)])[0]
        sig = session.sign(privKey, msg)
        # logout
        session.logout()
        session.closeSession()
        return sig

    def verify_certificate(self, sig_received, node_name, dig_received, cert):
        blk = hashlib.blake2s()
        with open(cert, 'rb') as f:
            while True:
                data = f.read(BUF_SIZE)
                if not data:
                    break
                blk.update(data)
        dig = blk.hexdigest()
        if dig == dig_received.decode('UTF-8'):
            return self.verify_hsm(dig_received, sig_received, node_name)

    def hashSig(self, cert):
        blk = hashlib.blake2s()
        with open(cert, 'rb') as stream:
            while True:
                data = stream.read(BUF_SIZE)
                if not data:
                    break
                blk.update(data)
            dig = blk.hexdigest()
            sig = self.sign_hsm(dig)
            return dig, sig
