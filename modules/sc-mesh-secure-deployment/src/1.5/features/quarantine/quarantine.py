import subprocess
import sys
sys.path.insert(0, '../')
from commom import mesh_utils


class Quarantine:
    def __int__(self, mac):
        self.mac = mac
        self.interface = mesh_utils.get_mesh_interface()

    def block(self, mac):
        command = ['traffic_block.sh', self.mac, self.interface]
        subprocess.call(command, shell=False)
        print(f'blocking MAC: {str(self.mac)} on interface {str(self.interface)}')
