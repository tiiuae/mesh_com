import argparse
import json
import subprocess
import time

import netifaces
import requests

# Construct the argument parser
ap = argparse.ArgumentParser()

# Add the arguments to the parser
ap.add_argument("-s", "--server", required=True, help="Server IP:Port Address. Ex: 'http://192.168.15.14:5000'")
ap.add_argument("-c", "--certificate", required=True)
args = ap.parse_args()

# URL = 'https://192.168.15.14:5000' #
URL = args.server
print(URL)


def get_os():
    proc = subprocess.Popen(['lsb_release', '-a'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = proc.communicate()
    for element in out.split():
        aux = element.decode('utf-8')
        if 'Ubuntu' in aux:
            os = aux
    return os


def get_data(cert_file, os):
    message = '/api/add_message/' + os
    response = requests.post(URL + message,
                             files={'key': open(cert_file, 'rb')})
    print('Encrypted Message: ' + str(response.content))

    with open('payload.enc', 'wb') as file:
        file.write(response.content)


def decrypt_reponse():  # assuming that data is on a file called payload.enc generated on the function get_data
    proc = subprocess.Popen(['src/ecies_decrypt', args.certificate], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = proc.communicate()
    aux_list = [element.decode() for element in out.split()]
    new_list = aux_list[40:64]
    joined_list = ''.join(new_list)
    output_dict = joined_list.replace("\'", '"')
    print('Decrypted Message: ', output_dict)  # res =  json.loads(output_dict)
    return output_dict


def create_config_openwrt(response):
    res = json.loads(response)
    if res['gateway']:
        config_file = open('/etc/config/lime-node', 'a+')
    else:
        nodeId = int(res['addr'].split('.')[-1]) - 1  # the IP is sequential, then it gaves me the nodeId.
        config_file = open('/etc/config/lime-node', 'w')
        config_file.write('config lime ' + "'system'" + '\n')
        config_file.write('\t' + 'option hostname ' + "'" + 'node' + str(nodeId) + "'" + '\n')
        config_file.write('\t' + 'option firstbootwizard_configured ' + "'" + 'true' + "'" + '\n\n')

    config_file.write('config wifi radio1' + '\n')
    config_file.write('\t' + 'list modes ' + "'ieee80211s'" + '\n')
    config_file.write('\t' + 'option ieee80211s_mesh_fwding ' + "'" + res['ieee80211s_mesh_fwding'] + "'" + '\n')
    config_file.write('\t' + 'option ieee80211s_mesh_id ' + "'" + res['ieee80211s_mesh_id'] + "'" + '\n')
    config_file.write('\t' + 'option channel ' + "'" + str(res['channel']) + "'" + '\n\n')

    config_file.write('config lime ' + "'network'" + '\n')
    config_file.write('\t' + 'list protocols ' + "'ieee80211s'" + '\n')
    config_file.write('\t' + 'list protocols ' + "'lan'" + '\n')
    config_file.write('\t' + 'list protocols ' + "'anygw'" + '\n')
    config_file.write('\t' + 'list protocols ' + "'" + 'batadv:' + res['batadv'] + "'" + '\n')
    config_file.write('\t' + 'list protocols ' + "'" + 'babeld:' + res['babeld'] + "'" '\n')
    config_file.write('\t' + 'list protocols ' + "'" + 'bmx7:' + res['bmx7'] + "'" + '\n')
    config_file.write('\t' + 'option main_ipv4_address ' + "'" + res['addr'] + '/24' + "'")

    config_file.close()


def get_interfaces():
    interface_list = netifaces.interfaces()
    interface = filter(lambda x: 'wlx' in x, interface_list)
    return list(interface)[0]


def ubuntu_gw():
    with open('/etc/systemd/system/gw.service', 'w') as new_config_file:
        new_config_file.write('[Unit]\n')
        new_config_file.write('Description="Gateway Service"\n\n')
        new_config_file.write('[Service]\n')
        new_config_file.write('Type=idle\n')
        command_gw = 'ExecStart=/usr/bin/run-gw.sh'
        subprocess.call('sudo cp src/client/run-gw.sh /usr/bin/.', shell=True)
        subprocess.call('sudo chmod 744 /usr/bin/run-gw.sh', shell=True)
        new_config_file.write(command_gw + '\n\n')
        new_config_file.write('[Install]\n')
        new_config_file.write('WantedBy=multi-user.target\n')
    subprocess.call('sudo chmod 644 /etc/systemd/system/gw.service', shell=True)
    subprocess.call('systemctl enable gw.service', shell=True)
    copy = 'sudo cp tools/wpa_tools/wpa_supplicant_client_AP.conf /etc/wpa_supplicant/wpa_supplicant-wlan0.conf'
    subprocess.call(copy, shell=True)
    subprocess.call('chmod 600 /etc/wpa_supplicant/wpa_supplicant-wlan0.conf', shell=True)
    subprocess.call('systemctl enable wpa_supplicant@wlan0.service', shell=True)
    subprocess.call('sudo sysctl -w net.ipv4.ip_forward=1', shell=True)
    subprocess.call('sudo iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE', shell=True) #assuming that wlan0 is the internet
    subprocess.call('sudo iptables -A FORWARD -i wlan0 -o bat0 -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT', shell=True)
    subprocess.call('sudo iptables -A FORWARD -i bat0 -o wlan0 -j ACCEPT', shell=True)


def create_config_ubuntu(response):
    interface = get_interfaces()
    res = json.loads(response)
    address = res['addr']
    prefix = address.split('.')[:-1]
    prefix = '.'.join(prefix)
    gw = prefix + '.2'
    with open('/etc/systemd/system/mesh.service', 'w') as config_file:
        config_file.write('[Unit]\n')
        config_file.write('Description="Mesh Service"\n\n')
        config_file.write('[Service]\n')
        config_file.write('Type=idle\n')
        command = 'ExecStart=/usr/local/bin/mesh_init.sh ' + address + ' ' + res['ap_mac'] + ' ' + res['key'] + ' ' \
                  + res['ssid'] + ' ' + res['frequency'] + ' ' + interface
        if res['gateway']:
            ubuntu_gw()
        else:
            default_route = 'route add default gw ' + gw + ' bat0'
            subprocess.call(default_route, shell=True)
            with open('/etc/systemd/system/default.service', 'w') as config_default:
                config_default.write('[Unit]\n')
                config_default.write('Description="IP Route Default Service"\n\n')
                config_default.write('[Service]\n')
                config_default.write('Type=idle\n')
                command_default = 'ExecStart=/usr/local/bin/default.sh '
                config_default.write(command_default + '\n\n')
                config_default.write('[Install]\n')
                config_default.write('WantedBy=multi-user.target\n')
            with open('/usr/local/bin/default.sh', 'w') as default_script:
                default_script.write('#!/bin/bash\n')
                default_script.write('sleep 5\n')
                default_script.write('ip route add default via ' + gw + '\n')
                subprocess.call('sudo chmod 744 /usr/local/bin/default.sh', shell=True)
                subprocess.call('sudo chmod 664 /etc/systemd/system/default.service', shell=True)
                subprocess.call('sudo systemctl enable default.service', shell=True)

        config_file.write(command + '\n\n')
        config_file.write('[Install]\n')
        config_file.write('WantedBy=multi-user.target\n')

    nodeId = int(res['addr'].split('.')[-1]) - 1  # the IP is sequential, then it gives the nodeId.
    command_hostname = 'sudo hostnamectl set-hostname node' + str(nodeId)
    subprocess.call(command_hostname, shell=True)
    command_hostname_host = 'echo ' + '"' + address + '\t' + 'node' + str(nodeId) + '"' + ' >' + '/etc/hosts'
    subprocess.call(command_hostname_host, shell=True)



def final_settings_ubuntu():
    # this setting assume that mesh_init.sh and run-gw.sh
    subprocess.call('sudo nmcli networking off', shell=True)
    subprocess.call('sudo systemctl stop network-manager.service', shell=True)
    subprocess.call('sudo systemctl disable network-manager.service', shell=True)
    subprocess.call('sudo systemctl disable wpa_supplicant.service', shell=True)
    subprocess.call('sudo cp src/client/mesh_init.sh /usr/local/bin/.', shell=True)
    subprocess.call('sudo chmod 744 /usr/local/bin/mesh_init.sh', shell=True)
    subprocess.call('sudo chmod 664 /etc/systemd/system/mesh.service', shell=True)
    subprocess.call('sudo systemctl enable mesh.service', shell=True)
    time.sleep(2)
    subprocess.call('reboot', shell=True)


def final_settings_openwrt():
    subprocess.call('lime-config', shell=True)
    time.sleep(2)
    subprocess.call('reboot', shell=True)


if __name__ == "__main__":
    os = get_os()
    get_data(args.certificate, os)
    res = decrypt_reponse()
    if os == 'Ubuntu':
        create_config_ubuntu(res)
        final_settings_ubuntu()
    if os == 'openwrt':
        create_config_openwrt(res)
        final_settings_openwrt()
