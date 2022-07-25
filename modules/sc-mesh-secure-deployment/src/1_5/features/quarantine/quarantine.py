import subprocess
import sys
sys.path.insert(0, '../../')
from common import mesh_utils


class Quarantine:
    def __int__(self):
        self.interface = mesh_utils.get_mesh_interface()

    def block(self, mac):
        command = ['./traffic_block.sh', mac, self.interface]
        subprocess.call(command, shell=False)
        print(f'blocking MAC: {str(self.mac)} on interface {str(self.interface)}')
