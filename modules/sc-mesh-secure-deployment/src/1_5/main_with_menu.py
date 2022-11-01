#! /usr/bin/python3

from new_main import *

menu_options = {
    1: 'Mutual Authentication (MA)',
    2: 'Only Mesh Network',
    3: 'Continuous Authentication (CA)',
    4: 'Continuous Authentication (CA) + Decision Engine',
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


def print_menu():
    banner()
    for key in menu_options.keys():
        print(key, '--', menu_options[key])


def aux_auth(semasphore):
    while True:
        with semasphore:
            mut = mutual.Mutual(MUTUALINT)
            client, addr = mut.server()
            sig, client_cert, cliID = mut.decode_cert(client)
            node_name = addr[0].replace('.', '_')
            pri.import_cert(client_cert, node_name)
            mut.send_my_key(cliID, addr)
            mut.cert_validation(sig, node_name, cliID, False, addr)


def option1():
    semasphore = threading.Semaphore(2)
    global AUTHSERVER
    print('\'Mutual Authentication\'')
    if not AUTHSERVER:  # already
        mutual_authentication()
        AUTHSERVER = True
    p = threading.Thread(target=aux_auth, daemon=True, args=(semasphore,))
    p.start()


def option2():
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


def option3():
    print('\'Continuous Authentication\'')
    if not mesh_utils.verify_mesh_status():  # verifying that mesh is running
        print("Mesh network not established")
        option2()
    return only_ca(mod=True)


def option4():
    print('\'Continuous Authentication + Decision Engine\'')
    sectable = option3()
    ness_result, first_round, mapp = decision_engine(True, {}, sectable, mba.MBA(mesh_utils.get_mesh_ip_address()),
                                                     queue.Queue())
    return ness_result, first_round, mapp


def option5():
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


def option6():
    os.system('clear')
    print('\'Check Neighbors Status\'')
    if mesh_utils.verify_mesh_status():  # verifying that mesh is running
        neigh = mesh_utils.get_macs_neighbors()
        for ne in neigh:
            print(f'{str(ne)} connected')
    else:
        print("No mesh established")


def option7():
    original_time = time()
    end_time = 60  # one minute
    sec_beat_time = 5
    os.system('clear')
    print('\'SecBeat\'')
    print(f'Running SecBeat every {sec_beat_time} seconds, during {end_time} minute')
    first_round = True  # flag for the decision engine. If not information available, create a table.
    ness_result = {}
    while time() < original_time + end_time:
        if mesh_utils.verify_mesh_status():  # verifying that mesh is running
            sleep(sec_beat_time - time() % sec_beat_time)  # sec beat time
            first_round, ness_result = sec_beat(first_round, ness_result)


def option8():
    os.system('clear')
    print('\'Exchange Security Table\'')
    table = 'auth/dev.csv'
    try:
        sectable = pd.read_csv(table)
        sectable.drop_duplicates(inplace=True)
        received = exchage_table(sectable, True)
        print(received)
        for ind in received:
            new = pd.read_json(received[ind].decode())
            sectable = pd.concat([sectable, new], ignore_index=True)
        sectable.drop_duplicates(inplace=True)
        sectable.to_csv(table, mode='w', header=True, index=False)
    except FileNotFoundError:
        print("SecTable not available. Need to be requested during provisioning")


def option9():
    os.system('clear')
    print('\'Quarantine\'')
    q = queue.Queue()

    # quarantine(ness_result, q, sectable, ma, mapp)


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
        if option == 1:
            option1()  # 1: 'Mutual Authentication (MA)',
        elif option == 2:
            option2()  # 2: 'Only Mesh Network',
        elif option == 3:
            option3()  # 3: 'Continuous Authentication (CA)',
        elif option == 4:
            option4()  # 4: 'Continuous Authentication (CA) + Decision Engine',
        elif option == 5:
            option5()  # 5: 'Show Security Table',
        elif option == 6:
            option6()  # 6: 'Check Neighbors',
        elif option == 7:
            option7()  # 7: 'SecBeat',
        elif option == 8:
            option8()  # 8: 'Exchange Table',
        elif option == 9:
            option8()  # 9: 'Quarantine',
        elif option == 0:
            print('Exiting....')
            # needs to close all the threads
            sys.exit()
        else:
            print('Invalid option. Please enter a number between 0 and 9.')
