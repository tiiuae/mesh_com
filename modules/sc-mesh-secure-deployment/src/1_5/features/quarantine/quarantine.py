import subprocess
import sys
import time

#script_path = pathlib.Path(__file__).parent.resolve()
sys.path.insert(0, '../../')
from common import mesh_utils, utils


class Quarantine:
    def __init__(self):
        self.interface = mesh_utils.get_mesh_interface(mesh_utils.get_mesh_int())

    def block(self, ip, quarantineTime):
        #command = [str(script_path) + '/traffic_block.sh', mac, self.interface]
        #command = ['iptables', '-A', 'INPUT', '-p', 'ALL', '-m', 'mac', '--mac-source', mac, '-j', 'DROP']
        command = ['iptables', '-I', 'INPUT', '-p', 'ALL', '-s', ip, '-j', 'DROP']
        subprocess.call(command, shell=False)
        command = ['iptables', '-I', 'FORWARD', '-p', 'ALL', '-s', ip, '-j', 'DROP']
        subprocess.call(command, shell=False)
        print(f'blocking IP: {str(ip)} on interface {str(self.interface)}')
        self.iptablesSave()
        for i in range(quarantineTime, 0, -1):
            print(f"Blocking IP: {str(ip)} for {str(i)} seconds", end='\r')
            time.sleep(1)
        self.unblock(ip)


    def unblock(self, ip):
        #command = [str(script_path) + '/traffic_block.sh', mac, self.interface]
        #command = ['iptables', '-D', 'INPUT', '-p', 'ALL', '-m', 'mac', '--mac-source', mac, '-j',  'DROP']
        command = ['iptables', '-D', 'INPUT', '-p', 'ALL', '-s', ip, '-j', 'DROP']
        subprocess.call(command, shell=False)
        command = ['iptables', '-D', 'FORWARD', '-p', 'ALL', '-s', ip, '-j', 'DROP']
        subprocess.call(command, shell=False)
        print(f'unblocking IP: {str(ip)} on interface {str(self.interface)}')
        self.iptablesSave()

    def iptablesSave(self):
        command = ['iptables-save', '>', 'iptables-', str(time.time()), '.save']
        subprocess.call(command, shell=False)

    def test(self):
        """
        unit test should be run as
        qua = quarantine.Quarantine()
        qua.test()
        stores log to logs/quarantine-log.txt
        """
        common_ut = utils.Utils()
        logger = common_ut.setup_logger('quarantine')
        blocking_time = 20 #in seconds

        ip = '127.0.0.1'
        self.printing('Blocking ', ip)
        self.block(ip, blocking_time)
        logger.info("Blocked %s", ip)
        command = ['ping', '-w', '4', ip]
        ping = subprocess.run(command, shell=False, capture_output=True, text=True)
        print(ping.stdout)
        logger.debug("ping %s output after blocking:\n%s", ip, ping.stdout)
        print('')
        print('IP Table:')
        command = ['iptables', '-L']
        iptable = subprocess.run(command, shell=False, capture_output=True, text=True)
        print(iptable.stdout)
        logger.debug("IP Table after blocking:\n%s", iptable.stdout)
        self.printing('Unblocking ', ip)
        self.unblock(ip)
        logger.info("Unblocked %s", ip)
        command = ['ping', '-w', '4', ip]
        ping = subprocess.run(command, shell=False, capture_output=True, text=True)
        print(ping.stdout)
        logger.debug("ping %s output after unblocking:\n%s", ip, ping.stdout)
        print('')
        print('IP Table:')
        command = ['iptables', '-L']
        iptable = subprocess.run(command, shell=False, capture_output=True, text=True)
        print(iptable.stdout)
        logger.debug("IP Table after unblocking:\n%s", iptable.stdout)
        common_ut.close_logger(logger)

    def printing(self, arg0, ip):
        print('***********************************************************************')
        print(arg0, ip)
        print('')
