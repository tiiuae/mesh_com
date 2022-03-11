#!/bin/python3 

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
import random
import ruamel.yaml


# Construct the argument parser
ap = argparse.ArgumentParser()
# Add the arguments to the parser
ap.add_argument("-c", "--clean", help='clean all (delete all keys and files)', required=False,
                default=False, const=True, nargs='?')
ap.set_defaults(clean=False)
args: argparse.Namespace = ap.parse_args()


def folder():
    if not path.isdir(PATH):
        os.mkdir(PATH)


PATH = '../auth/'
folder()
gpg = gnupg.GPG(gnupghome=PATH)
os.environ['GNUPGHOME'] = '../auth/'


def create_table():
    '''
    Function to create table of authenticated devices.
    '''
    columns = ['ID', 'MAC', 'IP', 'PubKey_fpr']
    if not path.isfile('../auth/dev.csv'):
        table = pd.DataFrame(columns=columns)
        table.to_csv('../auth/dev.csv', header=columns, index=False)
    else:
        table = pd.read_csv('../auth/dev.csv')
    return table


def set_auth_role():
    """
    To set the serve/client auth role.
    TODO: think a way to add more servers
    """
    file_name = '../../mesh_com.conf'
    config, ind, bsi = ruamel.yaml.util.load_yaml_guess_indent(open(file_name))
    config['client']['auth_role'] = 'server'
    print(config['client']['auth_role'])
    yaml = ruamel.yaml.YAML()
    yaml.indent(mapping=ind, sequence=ind, offset=bsi)
    with open('../../mesh_com.conf', 'w') as fp:
        yaml.dump(config, fp)


def update_table(info):
    '''
    this function update the table with the node's info.
    Then, it updates mesh_conf_file with ip address.
    Finally, it calls the transfer function.
    '''
    table = create_table()
    print(table)
    if len(table) == 1:  # first node to be added, then it will be server. == 1 means that provserver is there
        set_auth_role()
    if info['ID'] not in set(table['ID']):
        while info['IP'] in set(table['IP']):
            info['IP'] = '10.0.0.' + str(generate_ip().pop())
        table = table.append(info, ignore_index=True)
        table.drop_duplicates(inplace=True)
        table.to_csv('../auth/dev.csv', index=False)
    elif table.loc[table['ID'] == info['ID']]['PubKey_fpr'].all() != info['PubKey_fpr']:
        table = table.append(info, ignore_index=True)
        table.drop_duplicates(inplace=True)
        table.to_csv('../auth/dev.csv', index=False)
    update_conf_file(info['IP'])
    file_keys = '../auth/' + info['ID'] + '.asc'
    if info['ID'] != 'provServer':
        file_keys = '../auth/node' + info['ID'] + '.asc'
        sent = input("Would you like to send the certificates to a node? (yes/no): ")
        if sent in ['yes', 'YES', 'Yes', 'y', 'Y']:
            transfer(file_keys)


def update_conf_file(ip):
    '''
    Update the mesh_conf file with the new ip address.
    '''
    file_name = '../../mesh_com.conf'
    config, ind, bsi = ruamel.yaml.util.load_yaml_guess_indent(open(file_name))
    instances = config['server']['ubuntu']
    instances['ip'] = ip
    yaml = ruamel.yaml.YAML()
    yaml.indent(mapping=ind, sequence=ind, offset=bsi)
    with open('../../mesh_com.conf', 'w') as fp:
        yaml.dump(config, fp)


def exporting(ID, key, sign=True):
    '''
    This function export the public and private keys in a .asc file;
    If the ID is the server, it won't export.
    If the key is generated for the first time, it will be signed by the server key.
    '''
    ascii_armored_public_keys = gpg.export_keys(key)
    ascii_armored_private_keys = gpg.export_keys(key, True, expect_passphrase=False)
    file_keys = '../auth/' + ID + '.asc'
    if ID != 'provServer':
        file_keys_pub = '../auth/node' + ID + 'pb.asc'
        with open(file_keys_pub, 'w') as f:
            f.write(ascii_armored_public_keys)
        file_keys_pri = '../auth/node' + ID + 'pr.asc'
        with open(file_keys_pri, 'w') as f:
            f.write(ascii_armored_private_keys)
        if sign:
            '''
            Unfortunately python gnupg does not has key signing, only messages and files
            That's why it is implemented with a bash command.
            '''
            command = 'gpg --batch --yes --default-key provServer --sign-key ' + ID
            subprocess.call(command, shell=True)
            ascii_armored_public_keys = gpg.export_keys(key)
            with open(file_keys_pub, 'w') as f:
                f.write(ascii_armored_public_keys)
        print(ascii_armored_public_keys)
        print(ascii_armored_private_keys)
        print('\n---------------------------\n')
        print('Key pair stored in: ' + file_keys)
    else:
        with open(file_keys, 'w') as f:
            f.write(ascii_armored_public_keys)


def clean_all():
    keys = gpg.list_keys(True)
    for key in keys:
        gpg.delete_keys(key['fingerprint'], True, expect_passphrase=False)  # for private
        gpg.delete_keys(key['fingerprint'], expect_passphrase=False)  # for public
    files = glob.glob('../auth/*.asc')
    for fi in files:
        os.remove(fi)
    files = glob.glob('../auth/*.conf.gpg')
    for fi in files:
        os.remove(fi)
    try:
        os.remove('../auth/dev.csv')
    except FileNotFoundError:
        print('Nothing to clean')
    exit()


def exist(ID, verbose=True):
    '''
    Verify if the keys are presented in the keyring.
    If not, it will create new pair of keys.
    If exists, it will verify if the keys are still valid.
    '''
    public_keys = gpg.list_keys()
    realID = ID + ' <' + ID + '>'
    if ID != 'provServer':
        realID = 'node' + ID + ' <' + ID + '>'
    if realID not in public_keys.uids:
        return False, False
    index = public_keys.uids.index(realID)
    valid = float(public_keys[index]['expires'])
    if valid > time.time():
        if verbose:
            print('A key pair exist for node ' + ID + ' and it is valid until: ' +
                  str(datetime.datetime.utcfromtimestamp(valid)))
            exporting(ID, public_keys[index]['fingerprint'], False)
        return True, public_keys[index]['fingerprint']
    else:
        if verbose:
            print('A key pair exist for node ' + ID + ' but it expired on: ' +
                  str(datetime.datetime.utcfromtimestamp(valid)))
        return False, False


def generate(input_data, sign=True):
    '''
    Generate new keys based on the input data
    '''
    key = gpg.gen_key(input_data)
    fingerprint = key.fingerprint
    ID = input_data.split('Name-Real: ')[1].split('\n')[0]
    if 'node' in ID:
        ID = ID.split('node')[1]
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


def encrypt_conf(ID):
    '''
    Encrypt the mesh configuration file with the public key from the node (recipients)
    Save at ../auth/nodeID_conf.conf.gpg
    '''
    node = ID.split('../auth/')[1].split('.asc')[0]
    real = node + ' <' + node.split('node')[1] + '>'
    recipients = [
        ids['fingerprint'] for ids in gpg.list_keys() if real in ids['uids']
    ]
    output = ID + '_conf.conf.gpg'
    with open('../../mesh_com.conf', 'rb') as f:
        gpg.encrypt_file(f, recipients=recipients,
                         output=output)
    return output


def transfer(FILE):
    '''
    Transfer via scp the node certificate, server public key, and encrypted mesh_conf.
    '''
    server = input('Enter node IP address: ')
    username = 'root'
    password = 'root'  # assuming secure OS
    ssh = paramiko.SSHClient()
    ssh.load_host_keys(os.path.expanduser(os.path.join("~", ".ssh", "known_hosts")))
    ssh.connect(server, username=username, password=password)
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    sftp = ssh.open_sftp()
    only_node = FILE.split('../auth/')[1].split('.asc')[0]
    sftp.put(FILE.split('.asc')[0]+'pr.asc', '/hsm/' + only_node+'pr.asc')
    sftp.put(FILE.split('.asc')[0]+'pb.asc', '/hsm/' + only_node + 'pb.asc')
    sftp.put('../auth/provServer.asc', '/hsm/provServer.asc')
    encrypted_conf = encrypt_conf(FILE)
    sftp.put(encrypted_conf, '/hsm/' + encrypted_conf.split('../auth/')[1])
    sftp.close()
    ssh.close()


def generate_ip():
    return random.sample(range(2, 254), 2)


if __name__ == "__main__":
    '''
    The node ID is entered manually. Later this should be modified to receive it from provisioning
    '''
    if args.clean:
        clean_all()
    exists, fpr = exist('provServer', False)
    if not exists:
        input_data = gpg.gen_key_input(name_real='provServer',
                                       passphrase='',
                                       no_protection=True,
                                       name_email='provServer',
                                       expire_date='1m'
                                       )
        fpr = generate(input_data, False)

    mac: str = get_mac()  # only for server
    info = {'ID': 'provServer', 'MAC': mac, 'IP': '0.0.0.0', 'PubKey_fpr': fpr}
    update_table(info)
    ID, mac = get_id()
    exists, fpr = exist(ID)
    if not exists:
        input_data = gpg.gen_key_input(name_real='node' + str(ID),
                                       passphrase='',
                                       no_protection=True,
                                       name_email=ID,
                                       expire_date='1d'
                                       )
        fpr = generate(input_data)
    info = {'ID': ID, 'MAC': mac, 'IP': '10.0.0.' + str(generate_ip().pop()), 'PubKey_fpr': fpr}
    update_table(info)

    '''
    TODO: 
    * Managing keys: Delete in case they are compromised (manually?)
    * How IP will be obtained?? provided by the server? ->> done 
    * Convert to der certificates?
      
    '''
