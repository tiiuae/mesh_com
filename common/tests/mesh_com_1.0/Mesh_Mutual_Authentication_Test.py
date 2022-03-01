import time
import subprocess
import os
import re
from pexpect import pxssh



def get_macs():
    macs = []
    proc = subprocess.Popen(['batctl', 'n'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    i = 0
    for x in proc.stdout:
        aux = x.split()
        i += 1
        if i > 2:
            macs.append((aux[1]).decode("utf-8"))

    s = pxssh.pxssh()
    s.force_password = True
    hostname = "10.10.10.2"
    username = "root"
    password = "root"
    s.login(hostname, username, password)
    s.sendline('cat /sys/class/net/wlp*/address')  # run a command
    s.prompt()  # match the prompt
    var = str((s.before.decode('utf-8', 'ignore').splitlines()[1:]))
    # var = str(s.before.decode('utf-8', 'ignore'))
    var = re.sub(r"[\[\]]", "", var)
    var_new = var.replace("'", "")
    macs.append(var_new)
    # print(macs)
    s.logout()
    return macs


def format_file():
    with open("filemacs.json", "r") as fin:
        data = fin.read().splitlines(True)
    with open('filemacs.json', 'w') as fout:
        fout.writelines(data[3:-1])

    with open("filemacs.json", "r") as fin:
        data = fin.read()
    with open('filemacs.json', 'w') as fout:
        fout.writelines(re.sub(r"\s+", "", data))

    with open("filemacs.json", "r") as fin:
        data = fin.read()
    with open('filemacs.json', 'w') as fout:
        fout.writelines(data.replace(",", "\n"))

    with open("filemacs.json", "r") as fin:
        data = fin.read()
    with open('filemacs.json', 'w') as fout:
        fout.writelines(data.replace('"', ''))


def ping(macs):
    # Case Server pinging Clients
    f = open("filemacs.json", "w")
    print("Case Server Pinging Clients")

    s = pxssh.pxssh()
    s.force_password = True
    hostname = "10.10.10.2"
    username = "root"
    password = "root"
    s.login(hostname, username, password)
    s.sendline('cat /opt/container-data/mesh/mesh_com/modules/sc-mesh-secure-deployment/src/file.json')
    s.prompt()
    f.write(s.before.decode('utf-8', 'ignore'))
    f.close()

    format_file()
    s.logout()

    ip = {}
    with open("filemacs.json") as f:
        for line in f:
            (key, val) = line.split(':')
            ip[key] = val

    ip_addresses = []
    for key, value in ip.items():
        ip_addresses.append(key)



    m = open("Mesh_Mutual_result_file.log","w")
    m.write("Case Clients Pinging Each Other\n")
    process = subprocess.Popen('cat /sys/class/net/wlp*/address', stdout=subprocess.PIPE, shell=True)
    for x in process.stdout:
        var = str((x.decode('utf-8', 'ignore')))
        var = re.sub(r"[\[\]\n]", "", var)
        var_new = var.replace("'", "")
        print(var_new)
        m.write("Current Device MAC Address: \n")
        m.write(var_new)

    m.write("\n\nMAC Addresses of devices: \n")
    m.write(str(macs))
    m.write("\n\n\n")
    for x in range(len(macs) - 1):
        command = 'batctl ping -c 10 ' + macs[x]
        out = subprocess.check_output(command, shell=True)
        print('processing results of neighbor ' + str(x+1))
        m.write('processing results of neighbor ' + str(x+1))
        time.sleep(5)
        os.system("clear")
        output = out.decode("utf-8")
        if output != 'unreachable':
            output = output.split('\n')[-3:]
            # -1 is a blank line, -3 & -2 contain the actual results

            xmit_stats = output[0].split(",")
            timing_stats = output[1].split("=")[1].split("/")

            packet_loss = float(xmit_stats[2].split("%")[0])

            ping_min = float(timing_stats[0])
            ping_avg = float(timing_stats[1])
            ping_max = float(timing_stats[2])

            print('Results : \n')
            print('Packet loss: ', packet_loss)
            print('ping min: ', ping_min)
            print('ping max: ', ping_max)
            print('ping avg: ', ping_avg)
            print('\n')
            
            m.write('\nResults : \n')
            m.write('\nPacket loss: ')
            m.write(str(packet_loss))
            m.write('\nping min: ')
            m.write(str(ping_min))
            m.write('\nping max: ')
            m.write(str(ping_max))
            m.write('\nping avg: ')
            m.write(str(ping_avg))
            m.write('\n\n')
        else:
            print('Not reachable')
            m.write('\nNot reachable')
    
    m.write("Case Server Pining Clients\n")
    for x in range(len(ip_addresses)):
        s = pxssh.pxssh()
        s.force_password = True
        hostname = "10.10.10.2"
        username = "root"
        password = "root"
        s.login(hostname, username, password)
        m.write("Pinging Device with IP address: "+ str(ip_addresses[x] + "\n"))       
        print("Pinging Device with IP address: "+ str(ip_addresses[x] + "\n"))
        s.sendline("ping -c 5 "+str(ip_addresses[x]))
        s.prompt()
        print(s.before.decode('utf-8','ignore'))
        m.write(str(s.before.decode('utf-8','ignore')))
        m.write("\n\n\n")

    m.close()


if __name__ == '__main__':
    macs = get_macs()
    ping(macs)

