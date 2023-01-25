#! /usr/bin/python3

import contextlib
from main import *

menu_options = {
    1: 'Mutual Authentication (MA)',
    2: 'Only Mesh Network',
    3: 'Continuous Authentication (CA)',
    4: 'Decision Engine',
    5: 'Show Security Table',
    6: 'Check Neighbors',
    7: 'Security Beat',
    8: 'Exchange Table',
    9: 'Quarantine',
    0: "Exit"
}


def banner():
    print(r"""


      __  __        _      ___ _    _     _    _   _   ___ 
     |  \/  |___ __| |_   / __| |_ (_)___| |__| | / | | __|
     | |\/| / -_|_-< ' \  \__ \ ' \| / -_) / _` | | |_|__ \
     |_|  |_\___/__/_||_| |___/_||_|_\___|_\__,_| |_(_)___/
     ------------------------------------------------------  


    """)


AUTHSERVER = False
mut = mutual.Mutual(MUTUALINT)
myID = mut.myID

def print_menu():
    banner()
    for key in menu_options.keys():
        print(key, '--', menu_options[key])


def aux_auth(semasphore):
    while True:
        with semasphore:
            mut = mutual.Mutual(MUTUALINT)
            client, addr = mut.server()
            client_cert, sig, cliID, cli = mut.send_my_key(client, addr)
            #sig, client_cert, cliID = mut.decode_cert(client)
            node_name = addr[0].replace('.', '_')
            pri.import_cert(client_cert, node_name)
            #mut.send_my_key(cliID, addr)
            mut.cert_validation(sig, node_name, cliID, False, addr)

def MA():
    os.system('clear')
    semasphore = threading.Semaphore(2)
    global AUTHSERVER
    print('\'Mutual Authentication\'')
    if not AUTHSERVER:  # already
        mutual_authentication()
        AUTHSERVER = True
    p = threading.Thread(target=aux_auth, daemon=True, args=(semasphore,))
    p.start()


def only_mesh():
    os.system('clear')
    print('\'Run Only Mesh\'')
    mut = mutual.Mutual(MUTUALINT)
    try:
        mut.start_mesh()
        mut.create_table()
        sleep(2)
        if mesh_utils.verify_mesh_status():  # verifying that mesh is running
            print("Mesh Running")
            mesh_utils.get_neighbors_ip()
            neigh = mesh_utils.get_arp()
            for ne in neigh:
                info = {'ID': ne, 'MAC': neigh[ne], 'IP': ne,
                        'PubKey_fpr': "----", 'MA_level': 1}
                print(info)
                mut.update_table(info)
    except TypeError:
        print("No password provided for the mesh. Please get the password via provisioning server")
        exit()


def CA():
    os.system('clear')
    print('\'Continuous Authentication\'')
    sectable = pd.read_csv('auth/dev.csv')
    if not mesh_utils.verify_mesh_status():  # verifying that mesh is running
        print("Mesh network not established")
        only_mesh()
    return only_ca(myID)


def DE():
    os.system('clear')
    print('\'Decision Engine\'')
    try:
        sectable = pd.read_csv('auth/dev.csv')
        if 'CA_Result' not in sectable.columns:
            sectable = CA()
        if sectable.empty:
            print("Empty Security Table")
        ness_result, mapp = decision_engine(sectable, mba.MBA(mesh_utils.get_mesh_ip_address()),
                                            queue.Queue())
        return ness_result, mapp
    except FileNotFoundError:
        print("SecTable not available. Need to be requested during provisioning")


def show_table():
    os.system('clear')
    print('\'Current Security Table\'')
    try:
        sectable = pd.read_csv('auth/dev.csv')
        if not sectable.empty:
            print(sectable.drop_duplicates())
        else:
            print("Empty Security Table")
    except FileNotFoundError:
        print("SecTable not available. Need to be requested during provisioning")


def show_neighbors():
    os.system('clear')
    print('\'Check Neighbors Status\'')
    if mesh_utils.verify_mesh_status():  # verifying that mesh is running
        neigh = mesh_utils.get_macs_neighbors()
        for ne in neigh:
            print(f'{str(ne)} connected')
    else:
        print("No mesh established")


def sbeat():
    os.system('clear')
    original_time = time()
    end_time = 5  # one minute
    sec_beat_time = 5
    os.system('clear')
    print('\'SecBeat\'')
    print(f'Running SecBeat every {sec_beat_time} seconds, during {end_time} minutes')
    while time() < original_time + end_time:
        if mesh_utils.verify_mesh_status():  # verifying that mesh is running
            sleep(sec_beat_time - time() % sec_beat_time)  # sec beat time
            sec_beat(myID)


def extable():
    os.system('clear')
    print('\'Exchange Security Table\'')
    table = 'auth/dev.csv'
    try:
        mesh_utils.get_neighbors_ip()
        with contextlib.suppress(Exception):
            sectable = pd.read_csv(table)
            sectable.drop_duplicates(inplace=True)
            ut.exchage_table(sectable)
    except FileNotFoundError:
        print("SecTable not available. Need to be requested during provisioning")


def Quarantine():
    os.system('clear')
    print('\'Quarantine\'')
    ness_result, mapp = DE()
    q = queue.Queue()
    table = 'auth/global_table.csv'
    try:
        sectable = pd.read_csv(table)
        ma = mba.MBA(mesh_utils.get_mesh_ip_address())
        quaran(ness_result, q, sectable, ma, mapp)
    except FileNotFoundError:
        print("SecTable not available. Need to be requested during provisioning")


if __name__ == '__main__':
    wf.killall('wlan1')
    while True:
        print_menu()
        option = ''
        try:
            option = int(input('Enter your choice: '))
        except Exception:
            print('Wrong input. Please enter a number ...')
        # Check what choice was entered and act accordingly
        if option == 0:
            print('Exiting....')
            sys.exit()
        elif option == 1:
            MA()                # 1: 'Mutual Authentication (MA)',
        elif option == 2:
            only_mesh()         # 2: 'Only Mesh Network',
        elif option == 3:
            CA()                # 3: 'Continuous Authentication (CA)',
        elif option == 4:
            DE()                # 4: 'Decision Engine',
        elif option == 5:
            show_table()        # 5: 'Show Security Table',
        elif option == 6:
            show_neighbors()    # 6: 'Check Neighbors',
        elif option == 7:
            sbeat()             # 7: 'SecBeat',
        elif option == 8:
            extable()           # 8: 'Exchange Table',
        elif option == 9:
            Quarantine()        # 9: 'Quarantine',
        else:
            print('Invalid option. Please enter a number between 0 and 9.')
