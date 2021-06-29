import getopt
import sys
import datetime
import subprocess
from time import sleep
from threading import Thread

#defult rssi monitoring interval: 5sec
rssi_mon_interval = 5
#defualt interface
interface = "wlan0"

def get_rssi():
    global interface
    cmd = "iw dev " + interface + " station dump | grep 'signal:' | awk '{print $2}'"
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
    rssi = proc.communicate()[0].decode('utf-8')
    return rssi

def log_rssi():
    global rssi_mon_interval
    fn_suffix=str(datetime.datetime.now())
    log_file_path = '/var/log/'
    log_file_name =  'rssi'+fn_suffix+'.txt'
    while True:
        f = open(log_file_path+log_file_name, 'a')
        rssi_sta = get_rssi()
        print(rssi_sta)
        f.write(rssi_sta)
        f.close()
        sleep(rssi_mon_interval)

if __name__=='__main__':
    argv = sys.argv[1:]

    try:
        opts, args = getopt.getopt(argv, "hr:i:")
        for opt, arg in opts:
            if opt in ['-h']:
                print ("monitor_phy_param.py -r period -i interface")
                sys.exit()
            elif opt in ['-r']:
                print ("rssi mon period:", arg)
                rssi_mon_interval = int(arg)
            elif opt in ['-i']:
                print ("interface:", arg)
                interface = arg
    except getopt.error as err:
        print (str(err))

    Thread(target=log_rssi).start()

