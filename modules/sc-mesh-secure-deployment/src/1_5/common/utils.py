import subprocess
import threading
from os import path
import os as osh
import netifaces
import yaml
import pathlib
import logging

script_path = pathlib.Path(__file__).parent.resolve()

meshf = f'{str(script_path)}/mesh_com_11s.conf'


class Utils:
    def __init__(self):
        self.lock = threading.Lock()
        # name with absolute path : 'auth/dev.csv'
        # self.state_csv_file = self._conf['state_csv_name']
        # name with absolute path : '../../mesh_com.conf'
        # self.mesh_config_file = self._conf['mesh_conf_file']
        self.mesh_config_file = meshf

    @staticmethod
    def read_yaml(filename=meshf):
        with open(filename, 'r') as stream:
            try:
                return yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                return None

    # def init_state(self):
    #     """
    #     Function to create table of authenticated devices.
    #     TODO: Currently this function is inside mutual. Consider to export it here
    #     """
    #     columns = ['ID', 'MAC', 'IP', 'PubKey_fpr', 'MA_level']
    #     if not path.isfile(self.state_csv_file):
    #         table = pd.DataFrame(columns=columns)
    #         table.to_csv(self.state_csv_file, header=columns, index=False)
    #         table = pd.read_csv(self.state_csv_file)
    #     return table

    def update_mesh_conf(self, ip):
        """
        Update the mesh_conf file with the new ip address.
        """
        # config, ind, bsi = ruamel.yaml.util.load_yaml_guess_indent(open(self.mesh_config_file))
        config = self.read_yaml(self.mesh_config_file)

        instances = config['server']['secos']
        instances['ip'] = ip
        # yaml = ruamel.yaml.YAML()
        # yaml.indent(mapping=ind, sequence=ind, offset=bsi)
        with open(self.mesh_config_file, 'w') as fp:
            yaml.dump(config, fp)

    def update_mesh_password(self, password):
        '''
        Update the mesh_conf file with the password.
        '''
        config = self.read_yaml(self.mesh_config_file)
        instances = config['server']['secos']
        instances['key'] = password
        with open(self.mesh_config_file, 'w') as fp:
            yaml.dump(config, fp)

    @staticmethod
    def set_auth_role(self):
        """
        To set the serve/client auth role.
        TODO: think a way to add more servers
        """
        config = self.read_yaml(self.mesh_config_file)
        config['client']['auth_role'] = 'server'
        print(config['client']['auth_role'])
        with open(self.mesh_config_file, 'w') as fp:
            yaml.dump(config, fp)

    # def generate_ip(self):
    #     return self.random.sample(range(2, 254), 2)

    # def update_state(self, info):
    #     """
    #     this function update the table with the node's info.
    #     Then, it updates mesh_conf_file with ip address.
    #     Finally, it calls the transfer function.
    #     """
    #     table = self.init_state(self)
    #     print(table)
    #     if len(table) == 1:  # first node to be added, then it will be server. == 1 means that provserver is there
    #         self.set_auth_role(self)
    #     if info['ID'] not in set(table['ID']):
    #         while info['IP'] in set(table['IP']):
    #             info['IP'] = f'10.0.0.{str(self.generate_ip().pop())}'
    #         self.update_cont(table, info)
    #     elif table.loc[table['ID'] == info['ID']]['PubKey_fpr'].all() != info['PubKey_fpr']:
    #         self.update_cont(table, info)
    #     self.update_mesh_conf(info['IP'])
    #     if info['ID'] != 'provServer':
    #         return 'auth/node' + info['ID'] + '.asc'
    #     return None
    #
    # def update_cont(self, table, info):
    #     table = table.append(info, ignore_index=True)
    #     table.drop_duplicates(inplace=True)
    #     self.lock.acquire()
    #     table.to_csv(self.state_csv_file, index=False)
    #     self.lock.release()

    @staticmethod
    def get_os():  # this is not being used
        with subprocess.Popen(['lsb_release', '-a'], stdout=subprocess.PIPE, stderr=subprocess.PIPE) as proc:
            out, _ = proc.communicate()
            for element in out.split():
                aux = element.decode('utf-8')
                if 'Ubuntu' in aux:
                    oss = aux
            return oss

    @staticmethod
    def is_sec_os(self):  # this is not being used
        execution_ctx = osh.environ.get('EXECUTION_CTX')
        if execution_ctx == "docker":
            return "secos" if osh.environ.get('HOSTNAME') == "br_hardened" else self.get_os()
        return ""

    @staticmethod
    def get_interface_by_pattern(pattern):
        interface_list = netifaces.interfaces()
        interface = [x for x in interface_list if pattern in x]
        if pre := interface:
            return str(pre[0])
        print(f'> ERROR: Interface {pattern} not found!')
        return False

    @staticmethod
    def get_mac_by_interface(interface):
        mac = netifaces.ifaddresses(interface)[netifaces.AF_LINK]
        return mac[0]['addr']

    @staticmethod
    def set_hostname(id, mesh_ip):
        print('> Setting hostname...')
        command = ['hostname', 'node', id]
        subprocess.call(command, shell=False)
        command2 = ['echo', mesh_ip, 'node', id, '>', '/etc/hosts']
        subprocess.call(command2, shell=False)

    @staticmethod
    def setup_logger(name):
        if not path.isdir('logs'):
            osh.mkdir('logs')
        # create a logger object
        logger = logging.getLogger()
        logger.setLevel(logging.DEBUG)
        # create a file handler
        file_handler = logging.FileHandler(f'logs/{name}-log.txt', mode='w')
        file_handler.setLevel(logging.DEBUG)
        # create a formatter
        formatter = logging.Formatter(fmt='%(asctime)s %(levelname)-8s %(message)s',
                                      datefmt='%Y-%m-%d %H:%M:%S')
        # add the formatter to the file handler
        file_handler.setFormatter(formatter)
        # add the file handler to the logger
        logger.addHandler(file_handler)
        return logger

    @staticmethod
    def close_logger(logger):
        for handler in list(logger.handlers):
            handler.close()
            logger.removeHandler(handler)
