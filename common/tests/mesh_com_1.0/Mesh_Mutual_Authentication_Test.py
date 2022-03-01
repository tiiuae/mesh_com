import time
import subprocess
import os
from pexpect import pxssh


def get_macs():
    macs = []
    proc = subprocess.Popen(['batctl', 'n'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
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
    print(s.before)  # print everything before the prompt.

    macs.append(s.before)
    return macs


def ping(macs):

    # Case Server pinging Clients
    f = open("file.txt", "w")
    print("Case Server Pinging Clients")
    f.write("Case Server Pinging Clients\n\n")
    s = pxssh.pxssh()
    s.force_password = True
    hostname = "10.10.10.2"
    username = "root"
    password = "root"
    s.login(hostname, username, password)
    for x in range(len(macs) - 1):
        command = s.sendline('batctl ping -c 10' + macs[x])
        out = subprocess.check_output(command, shell=True)
        print('processing results for device' + str(x))
        f.write('processing results for device' + str(x))
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

            f.write('\nResults : \n')
            f.write('\nPacket loss: ')
            f.write(str(packet_loss))
            f.write('\nping min: ')
            f.write(str(ping_min))
            f.write('\nping max: ')
            f.write(str(ping_max))
            f.write('\nping avg: ')
            f.write(str(ping_avg))
            f.write('\n\n')
        else:
            print('Not reachable')
            f.write('\nNot reachable')

    # Case Clients pinging Clients
    f.write("\n\n\nCase Clients pining Clients")
    for x in range(len(macs) - 1):
        command = 'batctl ping -c 10 ' + macs[x]
        out = subprocess.check_output(command, shell=True)
        print('processing results')
        f.write("\nprocessing results")
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

            f.write('\nResults : \n')
            f.write('\nPacket loss: ')
            f.write(str(packet_loss))
            f.write('\nping min: ')
            f.write(str(ping_min))
            f.write('\nping max: ')
            f.write(str(ping_max))
            f.write('\nping avg: ')
            f.write(str(ping_avg))
            f.write('\n\n')
        else:
            print('Not reachable')
            f.write('\nNot reachable')
    f.close()


if __name__ == '__main__':
    macs = get_macs()
    ping(macs)
