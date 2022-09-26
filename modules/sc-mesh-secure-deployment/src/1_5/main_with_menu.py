from new_main import *
from features.mutual import mutual
import os
from features.mutual.utils import primitives as pri

menu_options = {
    1: 'Mutual Authentication (MA)',
    2: 'Only Mesh Network',
    3: 'Continuous Authentication (CA)',
    4: 'Continuous Authentication (CA) + Decision Engine',
    5: 'Show Security Table',
    6: 'Check Neighbors',
    7: "Exit"
}


def banner():
    print(r"""


      __  __        _      ___ _    _     _    _   _   ___ 
     |  \/  |___ __| |_   / __| |_ (_)___| |__| | / | | __|
     | |\/| / -_|_-< ' \  \__ \ ' \| / -_) / _` | | |_|__ \
     |_|  |_\___/__/_||_| |___/_||_|_\___|_\__,_| |_(_)___/
     ------------------------------------------------------  


    """)


def print_menu():
    banner()
    for key in menu_options.keys():
        print(key, '--', menu_options[key])


def aux_auth():
    while True:
        mut = mutual.Mutual(mutual_int)
        client, addr = mut.server()
        sig, client_cert, cliID = mut.decode_cert(client)
        node_name = addr[0].replace('.', '_')
        pri.import_cert(client_cert, node_name)
        mut.cert_validation(sig, node_name, cliID, False, addr)


def option1():
    print('\'Mutual Authentication\'')
    mutual_authentication()
    p = threading.Thread(target=aux_auth, daemon=True, args=())
    p.start()


def option2():
    print('\'Run Only Mesh\'')
    mut = mutual.Mutual(mutual_int)
    try:
        mut.start_mesh()
    except TypeError:
        print("No password provided for the mesh. Please get the password via provisioning server")
        exit()


def option3():
    print('\'Continuous Authentication\'')
    if not mesh_utils.verify_mesh_status():  # verifying that mesh is running
        print("Mesh network not established")
        option2()
    try:
        sectable = pd.read_csv('auth/dev.csv')
        if not sectable.empty:
            sectable = continuous_authentication(sectable)
            received = exchage_table(sectable)
            for ind in received:
                new = pd.read_json(received[ind].decode())
                sectable = pd.concat([sectable, new], ignore_index=True)
            return sectable
        else:
            print("Empty Security Table")
            exit()
    except FileNotFoundError:
        print("SecTable not available. Need to be requested during provisioning")


def option4():
    print('\'Continuous Authentication + Decision Engine\'')
    sectable = option3()
    ness_result, first_round, mapp = decision_engine(True, {}, sectable, mba.MBA(mesh_utils.get_mesh_ip_address()),
                                                     queue.Queue())


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
            print(str(ne) + ' connected')
    else:
        print("No mesh established")


if __name__ == '__main__':
    while True:
        print_menu()
        option = ''
        try:
            option = int(input('Enter your choice: '))
        except Exception:
            print('Wrong input. Please enter a number ...')
        # Check what choice was entered and act accordingly
        if option == 1:
            option1()  # 'Mutual Authentication (MA)',
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
            print('Exiting....')
            exit()
        else:
            print('Invalid option. Please enter a number between 1 and 7.')
