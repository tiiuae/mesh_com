from PyKCS11 import *
import subprocess
import hashlib
import os

import base64
from cryptography.fernet import Fernet
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.kdf.pbkdf2 import PBKDF2HMAC

BUF_SIZE = 65536  # lets read stuff in 64kb chunks!
LIB = "/usr/lib/softhsm/libsofthsm2.so"
os.environ['PYKCS11LIB'] = LIB

debug = True
mechanism = Mechanism(CKM_ECDSA, None) # Mechanism for EC sign and verify

'''
TODO:create a function to verify if keys exist and are valid
'''


def get_session():
    pkcs11 = PyKCS11Lib()
    pkcs11.load()  # define environment variable PYKCS11LIB=YourPKCS11Lib
    slot = pkcs11.getSlotList(tokenPresent=True)[0]
    try:
        session = pkcs11.openSession(slot, CKF_SERIAL_SESSION | CKF_RW_SESSION)
        session.login("1234")
        return session
    except PyKCS11Error:
        print('No previous keys')
        return False
    # get 1st slot


def exit():
    clean_all()  # this will delete all keys
    session = get_session()
    session.logout()
    session.closeSession()

def derive_ecdh_secret(node_name, cliID):
    '''
    Derive the shared secret for EC keys
    '''
    pubKey_filename = f'{node_name}.der'
    if not os.path.exists('secrets/'):
        os.mkdir('secrets/')
    secret_filename = f'secrets/secret_{cliID}.der'
    command = ['pkcs11-tool', '--module', LIB, '-l', '--pin', '1234', '--id', '01', '--derive', '-i', pubKey_filename, '--mechanism', 'ECDH1-DERIVE', '--output-file', secret_filename]
    subprocess.call(command, shell=False)

'''
def encrypt_response(message,
                     name):  # assuming that data is on a file called payload.enc generated on the function get_data
    session = get_session()
    keys = session.findObjects()
    for key in range(len(keys)):
        aux = keys[key].to_dict()
        if aux['CKA_LABEL'] == name:
            pubKey = keys[key]
    encr = session.encrypt(pubKey, message)
    session.logout()
    session.closeSession()
    if debug:
        print(f'> Encrypted message: {bytes(encr)}')
    # logout
    return bytes(encr)

def decrypt_response(encr):  # assuming that data is on a file called payload.enc generated on the function get_data
    session = get_session()
    privKey = session.findObjects([(CKA_CLASS, CKO_PRIVATE_KEY)])[0]
    dec = session.decrypt(privKey, encr)
    if type(dec) == 'PyKCS11.ckbytelist':
        dec = bytes(dec)
    session.logout()
    session.closeSession()
    if debug:
        print(f'> Decrypted message: {bytes(dec)}')
    # logout
    return dec
'''

def encrypt_response(message, cliID):
    secret_filename = f'secrets/secret_{cliID}.der'
    secret = open(secret_filename, 'rb')
    password = secret.read()
    #salt = os.urandom(16)
    salt = bytes('', 'utf-8')
    kdf = PBKDF2HMAC(
        algorithm=hashes.SHA256(),
        length=32,
        salt=salt,
        iterations=390000,
    )
    key = base64.urlsafe_b64encode(kdf.derive(password))
    f = Fernet(key)
    encr = f.encrypt(bytes(message, 'utf-8')) # Encrypted message in bytes
    if debug:
        print("key: ", key)
        print(f'> Encrypted message: {encr}')
    return encr

def decrypt_response(encr, cliID):
    secret_filename = f'secrets/secret_{cliID}.der'
    secret = open(secret_filename, 'rb')
    password = secret.read()
    #salt = os.urandom(16)
    salt = bytes('', 'utf-8')
    kdf = PBKDF2HMAC(
        algorithm=hashes.SHA256(),
        length=32,
        salt=salt,
        iterations=390000,
    )
    key = base64.urlsafe_b64encode(kdf.derive(password))
    f = Fernet(key)
    dec = f.decrypt(encr) # Decrypted message in bytes
    if debug:
        print("key: ", key)
        print(f'> Decrypted message: {dec}')
    return dec


def sign_hsm(msg):
    session = get_session()
    privKey = session.findObjects([(CKA_CLASS, CKO_PRIVATE_KEY)])[0]
    #privKey = session.findObjects([(CKA_CLASS, CKO_PRIVATE_KEY), (CKA_KEY_TYPE, CKK_ECDSA)])[0]
    sig = session.sign(privKey, msg, mechanism)
    # logout
    session.logout()
    session.closeSession()
    return sig


def delete_key(node_name):  # check if it is possible to delete on python (destroy_objetc)
    label = f'--label={node_name}'
    k = []
    session = get_session()
    keys = session.findObjects()
    for key in range(len(keys)):
        aux = keys[key].to_dict()
        if aux['CKA_LABEL'] == node_name:
            aux = keys[key].to_dict()
            k.append(aux)
    for _ in range(len(k)):
        command = ['pkcs11-tool', '--module', LIB, '--delete-object', label, '--type=pubkey']
        subprocess.call(command, shell=False)


def verify_key_exists(node_name):
    k = []
    session = get_session()
    keys = session.findObjects()
    for key in range(len(keys)):
        aux = keys[key].to_dict()
        if aux['CKA_LABEL'] == node_name:
            aux = keys[key].to_dict()
            k.append(aux)
    if k:
        return True


def import_cert(client_key, node_name):
    exist = verify_key_exists(node_name)
    if exist:
        delete_key(node_name)
    filename = f'{node_name}.der'
    if node_name == 'root':
        filename = client_key
        ID = str(999)
    else:
        ID = node_name.split('_')[-1]
        try:
            with open(filename, 'wb') as writer:
                writer.write(client_key.read())
        except AttributeError:
            with open(filename, 'wb') as writer:
                writer.write(client_key)
    command = ['pkcs11-tool', '--module', LIB, '-l', '--pin', '1234', '--write-object', filename, '--type',
               'pubkey', '--id', ID, '--label', node_name]
    subprocess.call(command, shell=False)


def verify_hsm(msg, sig, name):
    session = get_session()
    keys = session.findObjects()
    for key in range(len(keys)):
        aux = keys[key].to_dict()
        if aux['CKA_LABEL'] == name:
            pubKey = keys[key]  ##check here need to verify if not exported
    ver = session.verify(pubKey, msg, sig, mechanism)
    # logout
    session.logout()
    session.closeSession()
    return ver


def verify_certificate(sig_received, node_name, dig_received, cert):
    blk = hashlib.blake2s()
    with open(cert, 'rb') as f:
        while True:
            data = f.read(BUF_SIZE)
            if not data:
                break
            blk.update(data)
    dig = blk.hexdigest()
    if dig == dig_received:  # decode('UTF-8'):
        return verify_hsm(dig_received, sig_received, node_name)


def hashSig(cert):
    blk = hashlib.blake2s()
    with open(cert, 'rb') as stream:
        while True:
            data = stream.read(BUF_SIZE)
            if not data:
                break
            blk.update(data)
        dig = blk.hexdigest()
        sig = sign_hsm(dig)
        return dig, sig


def clean_all():
    session = get_session()
    if session:
        keys = session.findObjects()
        for key in range(len(keys)):
            session.destroyObject(keys[key])


def chunks(file_obj, size=10000):
    counter, chunks = 0, []
    for line in file_obj:
        if line == '\n':
            continue
        counter += 1
        chunks.append(line)
        if counter == size:
            yield chunks
            counter, chunks = 0, []
    file_obj.close()
    if counter:
        yield chunks


def encrypt_file(file_name, key_name):
    encr = []
    session = get_session()
    keys = session.findObjects()
    for key in range(len(keys)):
        aux = keys[key].to_dict()
        if aux['CKA_LABEL'] == key_name:
            pubKey = keys[key]
    split_files = chunks(open(file_name))
    for chunk in split_files:
        size = len(chunk)
    for ch in range(size):
        c = session.encrypt(pubKey, chunk[ch])
        if debug:
            print(bytes(c))
        encr.append(bytes(c))
    session.logout()
    session.closeSession()
    return encr


def decrypt_file(file_name):
    ''''
    not working
    '''
    decr = []
    session = get_session()
    privKey = session.findObjects([(CKA_CLASS, CKO_PRIVATE_KEY)])[0]
    split_files = chunks(open(file_name))
    for chunk in split_files:
        size = len(chunk)
    for ch in range(size):
        c = session.decrypt(privKey, bytes(chunk[ch].split('\n')[0], encoding='utf-8'))
        if debug:
            print(bytes(c))
        decr.append(bytes(c))
    session.logout()
    session.closeSession()
    return decr


def get_labels():
    labels = []
    session = get_session()
    keys = session.findObjects()
    for key in range(len(keys)):
        aux = keys[key].to_dict()
        if aux['CKA_CLASS'] == 'CKO_PRIVATE_KEY':
            labels.append(aux['CKA_LABEL'])
    return list(set(labels))[0]


def main():
    clean_all()


if __name__ == "__main__":
    main()