import time
import subprocess
import os


def get_macs():
    macs = []
    proc = subprocess.Popen(['batctl', 'n'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    i = 0
    for x in proc.stdout:
        aux = x.split()
        i += 1
        if i > 2:
            macs.append((aux[1]).decode("utf-8"))
    return macs


def get_interfaces():
    interfaces = []
    proc = subprocess.Popen(['batctl', 'n'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    i = 0
    for x in proc.stdout:
        aux = x.split()
        i += 1
        if i > 2:
            interfaces.append((aux[0]).decode("utf-8"))
            interfaces = list(dict.fromkeys(interfaces))
    return interfaces


def statistics():
    proc = subprocess.Popen(['batctl', 's'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = proc.communicate()
    os.system("clear")
    print('Results : \n')
    print(out.split())
    print('\n')
    main()


def ping():
    mac = nodes()
    command = 'batctl ping -c 10 ' + mac
    out = subprocess.check_output(command, shell=True)
    print('processing results')
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
    else:
        print('Not reachable')
    main()


def troughput():
    mac = nodes()
    command = 'batctl tp -t 60000 ' + mac
    out = subprocess.check_output(command, shell=True)
    print('processing results')
    os.system("clear")
    print('Results :')
    output = out.decode("utf-8")
    print(output)
    print('\n')
    main()


def traceroute():
    mac = nodes()
    command = 'batctl tr ' + mac
    out = subprocess.check_output(command, shell=True)
    print('processing results')
    os.system("clear")
    print('Results :')
    output = out.decode("utf-8")
    print(output)
    print('\n')
    main()


def tcpdump():
    interface = get_interfaces()
    if len(interface) > 1:
        print('Available Interfaces: ')
        for node in range(len(interface)):
            print(node + 1 + ' ' + interface[node])
        dev = input("\nEnter the interface you would like to test : ")
    else:
        dev = interface[0]
    p = subprocess.Popen(["batctl", 'td', dev], stdout=subprocess.PIPE, bufsize=1)
    try:
        for line in iter(p.stdout.readline, b''):
            print(line.decode("utf-8"))
        p.stdout.close()
        p.wait()
    except KeyboardInterrupt:
        main()


def nodes():
    os.system("clear")
    print('Available nodes: ')
    for node in range(len(macs)):
        print(str(node + 1) + ': ' + macs[node])
    dev = input("\nEnter the node you would like to test : ")
    return macs[int(dev) - 1]


def main():
    print('Menu :\n')
    print("\nChoose test you want to use : ")
    print("""
        1 : Statics
        2 : ping during 10 seconds
        3 : Throughput meter during 1 minute
        4 : traceroute
        5 : tcpdump
        0 : Exit"""
          )
    choice = input("\nEnter your choice : ")
    if choice == '1':
        statistics()
    elif choice == '2':
        ping()
    elif choice == '3':
        troughput()
    elif choice == '4':
        traceroute()
    elif choice == '5':
        tcpdump()
    elif choice == '0':
        exit()
    os.system("clear")


if __name__ == '__main__':
    macs = get_macs()
    main()