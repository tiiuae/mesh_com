import datetime
import os
import time
from os import path
import subprocess
import hashlib
import gnupg
import pandas as pd
import netifaces
import paramiko
import glob
import argparse


# Construct the argument parser
ap = argparse.ArgumentParser()
# Add the arguments to the parser
ap.add_argument("-c", "--clean", help='clean all (delete all keys and files)', required=False)
args = ap.parse_args()


def folder():
    if not path.isdir(PATH):
        os.mkdir(PATH)


PATH = 'auth/'
folder()
gpg = gnupg.GPG(gnupghome=PATH)
os.environ['GNUPGHOME'] = 'auth/'


def create_table():
    '''
    Function to create table of authenticated devices
    '''
    columns = ['ID', 'MAC', 'IP', 'PubKey_fpr']
    if not path.isfile('auth/dev.csv'):
        table = pd.DataFrame(columns=columns)
        table.to_csv('auth/dev.csv', header=columns, index=False)
    else:
        table = pd.read_csv('auth/dev.csv')
    return table


def update_table(info):
    '''
    this function update the table with the node's info
    '''
    table = create_table()
    if info['ID'] not in set(table['ID']):
        table = table.append(info, ignore_index=True)
        table.to_csv('auth/dev.csv', mode='a', header=False, index=False)
    elif table.loc[table['ID'] == info['ID']]['PubKey_fpr'] != info['PubKey_fpr']:
        table = table.append(info, ignore_index=True)
        table.to_csv('auth/dev.csv', mode='a', header=False, index=False)


def exporting(ID, key, sign=True):
    '''
    This function export the public and private keys in a .asc file;
    If the ID is the server, it won't export.
    If the key is generated for the first time, it will be signed by the server key.
    '''
    ascii_armored_public_keys = gpg.export_keys(key)
    ascii_armored_private_keys = gpg.export_keys(key, True, expect_passphrase=False)
    file_keys = ID + '.asc'
    if ID != 'provServer':
        file_keys = 'auth/node' + ID + '.asc'
    with open(file_keys, 'w') as f:
        f.write(ascii_armored_public_keys)
        f.write(ascii_armored_private_keys)
    if sign:
        '''
        Unfortunately python gnupg does not has key signing, only messages and files
        That's why it is implemented with a bash command.
        '''
        ascii_armored_public_keys = gpg.export_keys(key)
        command = 'gpg --batch --yes --default-key provServer --sign-key ' + ID
        subprocess.call(command, shell=True)
        with open(file_keys, 'w') as f:
            f.write(ascii_armored_public_keys)
            f.write(ascii_armored_private_keys)
    print(ascii_armored_public_keys)
    print(ascii_armored_private_keys)
    print('\n---------------------------\n')
    print('Key pair stored in: ' + file_keys)
    sent = input("Would you like to send the certificates to a node? (yes/no): ")
    if sent in ['yes', 'YES', 'Yes', 'y', 'Y']:
        transfer(file_keys)


def clean_all():
    keys = gpg.list_keys(True)
    for key in keys:
        gpg.delete_keys(key['fingerprint'], True, expect_passphrase=False) # for private
        gpg.delete_keys(key['fingerprint'], expect_passphrase=False)  # for public
    files = glob.glob('auth/*.asc')
    for fi in files:
        os.remove(fi)
    os.remove('auth/dev.csv')


def exist(ID, verbose=True):
    '''
    Verify if the keys are presented in the keyring.
    If not, it will create new pair of keys.
    If exist, it will verify if the keys are still valid.
    '''
    public_keys = gpg.list_keys()
    realID = ID + ' <' + ID + '>'
    if ID != 'provServer':
        realID = 'node' + ID + ' <' + ID + '>'
    if realID not in public_keys.uids:
        return False
    index = public_keys.uids.index(realID)
    valid = float(public_keys[index]['expires'])
    if valid - time.time() > 0:
        if verbose:
            print('A key pair exist for node ' + ID + ' and it is valid until: ' +
                  str(datetime.datetime.utcfromtimestamp(valid)))
            exporting(ID, public_keys[index]['fingerprint'], False)
        return True
    else:
        if verbose:
            print('A key pair exist for node ' + ID + ' but it expired on: ' +
                  str(datetime.datetime.utcfromtimestamp(valid)))
        return False


def generate(input_data, export=True, sign=True):
    '''
    Generate new keys based on the input data
    '''
    key = gpg.gen_key(input_data)
    fingerprint = key.fingerprint
    if export:
        print('Key pair generated with fingerprint ' + fingerprint)
        if not sign:
            exporting(ID, fingerprint, False)
        else:
            exporting(ID, fingerprint)
    return fingerprint


def get_id():
    '''
    Small function to get an ID.
    Determined from the hash of the mac address
    '''
    mac = input("Enter a MAC Address (in the format AA:BB:CC:DD:EE) : ")
    aux = hashlib.blake2s(mac.encode(), digest_size=2)
    baux = aux.hexdigest()
    ID = int(baux, 16)
    return str(ID), mac


def get_mac():
    for interf in netifaces.interfaces():
        # TODO: what it if doesn't start with wlan???
        if interf.startswith('wl'):
            interface = interf
            mac = netifaces.ifaddresses(interface)[netifaces.AF_LINK]
            return mac[0]['addr']


def transfer(FILE):
    server = input('Enter node IP address: ')
    username = 'root'
    password = 'root'
    ssh = paramiko.SSHClient()
    ssh.load_host_keys(os.path.expanduser(os.path.join("~", ".ssh", "known_hosts")))
    ssh.connect(server, username=username, password=password)
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    sftp = ssh.open_sftp()
    sftp.put(FILE, 'hsm/'+FILE.split('auth/')[1])
    sftp.close()
    ssh.close()


if __name__ == "__main__":
    '''
    The node ID is entered manually. Later this should be modified to receive it from provisioning
    '''
    if args.clean:
        clean_all()
    if not exist('provServer', False):
        input_data = gpg.gen_key_input(name_real='provServer',
                                       passphrase='',
                                       no_protection=True,
                                       name_email='provServer',
                                       expire_date='1m'
                                       )
        fpr = generate(input_data, False, False)
        mac = get_mac()  # only for server
        info = {'ID': 'provServer', 'mac': mac, 'IP': '0.0.0.0', 'PubKey_fpr': fpr}
        update_table(info)
    ID, mac = get_id()
    if not exist(ID):
        input_data = gpg.gen_key_input(name_real='node' + str(ID),
                                       passphrase='',
                                       no_protection=True,
                                       name_email=ID,
                                       expire_date='1d'
                                       )
        fpr = generate(input_data)
        info = {'ID': ID, 'mac': mac, 'IP': '0.0.0.0', 'PubKey fpr': fpr}
        update_table(info)

        '''
        TODO: 
        * Managing keys: Delete in case they are compromised (manually?)
        * Sign provisioning file??      
        * How IP will be obtained?? provided by the server?
        * Convert to der certificates?
          
        '''
