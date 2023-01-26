import subprocess
import sys
import time

#script_path = pathlib.Path(__file__).parent.resolve()
sys.path.insert(0, '../../')
from common import mesh_utils


class Quarantine:
    def __init__(self):
        self.interface = mesh_utils.get_mesh_interface('bat0')

    def block(self, ip):
        #command = [str(script_path) + '/traffic_block.sh', mac, self.interface]
        #command = ['iptables', '-A', 'INPUT', '-p', 'ALL', '-m', 'mac', '--mac-source', mac, '-j', 'DROP']
        command = ['iptables', '-I', 'INPUT', '-p', 'ALL', '-s', ip, '-j', 'DROP']
        subprocess.call(command, shell=False)
        print(f'blocking IP: {str(ip)} on interface {str(self.interface)}')
        self.iptablesSave()


    def unblock(self, ip):
        #command = [str(script_path) + '/traffic_block.sh', mac, self.interface]
        #command = ['iptables', '-D', 'INPUT', '-p', 'ALL', '-m', 'mac', '--mac-source', mac, '-j',  'DROP']
        command = ['iptables', '-D', 'INPUT', '-p', 'ALL', '-s', ip, '-j', 'DROP']
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
        stores log to test_logs/quarantine.txt
        """
        self.block('127.0.0.1')
        command = ['ping', '-w', '4', '127.0.0.1']
        subprocess.call(command, shell=False)
        self.unblock('127.0.0.1')
        command = ['ping', '-w', '4', '127.0.0.1']
        subprocess.call(command, shell=False)