import subprocess
import os as osh
import ruamel.yaml
import random
import string
import netifaces



def get_interface(pattern):
    '''
    Using this function from previous script, to obtain the mesh_interface.
    Maybe it's redundant if secure OS will have 'wlan1' as default
    '''
    interface_list = netifaces.interfaces()
    interface = filter(lambda x: pattern in x, interface_list)
    pre = list(interface)
    if not pre:
        print('> ERROR: Interface ' + pattern + ' not found!')
    else:
        return pre[0]


def update_password(password):
    '''
    Update the mesh_conf file with the password.
    '''
    file_name = 'src/mesh_com.conf'
    config, ind, bsi = ruamel.yaml.util.load_yaml_guess_indent(open(file_name))
    instances = config['server']['ubuntu']
    instances['key'] = password
    yaml = ruamel.yaml.YAML()
    yaml.indent(mapping=ind, sequence=ind, offset=bsi)
    with open('src/mesh_conf.conf', 'w') as fp:
        yaml.dump(config, fp)


def create_mesh(ID):    # Get the mesh_com config
    '''
    load mesh_conf as yaml file
    '''
    print('> Loading yaml conf... ')
    try:  # may be this is redundant, since we'll always have the file.
        yaml = ruamel.yaml.YAML(typ='safe')  # default, if not specfied, is 'rt' (round-trip)
        doc = open('src/mesh_com.conf', 'r')
        yaml_conf = yaml.load(doc.read())
        conf = yaml_conf['client']
        debug = yaml_conf['debug']
        print(conf)
    except (IOError, yaml.YAMLError) as error:
        print(error)
        exit()

    if conf['set_hostname']:
        print('> Setting hostname...')
        subprocess.call('hostname node' + ID, shell=True)
        subprocess.call('echo ' + '"' + conf['address'] + '\t' + 'node' + ID + '"' + ' >' + '/etc/hosts', shell=True)
    execution_ctx = osh.environ.get('EXECUTION_CTX')
    print("EXECUTION CTX:")
    print(execution_ctx)
    if execution_ctx != "docker" and conf['disable_networking']:
        subprocess.call('sudo nmcli networking off', shell=True)
        subprocess.call('sudo systemctl stop network-manager.service', shell=True)
        subprocess.call('sudo systemctl disable network-manager.service', shell=True)
        subprocess.call('sudo systemctl disable wpa_supplicant.service', shell=True)
    # Copy mesh service to /etc/systemd/system/
    if conf['mesh_service']:
        mesh_interface = get_interface(conf['mesh_inf'])
        if conf['type'] == '11s':
            subprocess.call('src/bash/conf-11s-mesh.sh ' + mesh_interface, shell=True)
        if conf['type'] == 'ibss':
            subprocess.call('src/bash/conf-mesh.sh ' + mesh_interface, shell=True)


def create_password():
    '''
    get random password pf length 8 with letters, digits
    '''
    characters: str = string.ascii_letters + string.digits
    return ''.join(random.choice(characters) for i in range(8))