from .ca_main import *
from ..mutual.mutual import *

sys.path.append('../../..')
from common import ConnectionMgr, mesh_utils

co = ConnectionMgr.ConnectionMgr()
MUTUALINT = 'wlan1'
MESHINT = mesh_utils.get_mesh_interface_from_file()

client_q = {}


async def launchCA(sectable):
    """
    this function should start server within localhost and client within the neighbors
    It should be the trigger in time-bases ex: every X seconds
    """
    ID = random.randint(1000, 64000)
    with contextlib.suppress(OSError):
        ca = CA(ID)
    myip =  mesh_utils.get_mesh_ip_address(MESHINT)

    manager = multiprocessing.Manager()
    return_dict = manager.dict()
    id_dict = manager.dict()

    neighbor_ips = mesh_utils.get_neighbors_ip()
    ip2_send = list(set(sectable['IP'].tolist() + neighbor_ips))
    if myip in ip2_send: ip2_send.remove(myip) # remove node's own IP

    #ip2_send = list(set(sectable['IP'].tolist() + list(neigh.keys())))
    print('Checkpoint, neighbor IPs = ', neighbor_ips)
    print('Checkpoint, ip2send = ', ip2_send)
    neigh = mesh_utils.get_arp()
    print('Checkpoint, IP_get_arp = ', list(neigh.keys()))

    num_neighbors = len(ip2_send) # Number of neighbor nodes that need to be authenticated
    server_proc = ca_server(myip, return_dict, id_dict, num_neighbors, list(set(sectable['IP'].tolist())))

    client_procs = []
    for IP in ip2_send:
        if IP != myip:
            print("neighbor IP:", IP)
            proc = multiprocessing.Process(target=ca.as_client, args=(IP,), daemon=True)
            client_procs.append(proc)
            proc.start()

    for proc in client_procs:
        proc.join()

    #server_proc.terminate()
    server_proc.join()
    print("Checkpoint server proc terminating")

    for key in return_dict.keys():
        count = np.unique(return_dict[key], return_counts=True)
        try:
            if len(count[1]) > 1:
                final = count[0][0] if count[1][0] > count[1][1] else count[0][1]
            elif len(count[1]) == 0:
                # Empty result
                final = 0
            else:
                final = count[0][0]
        except IndexError:
            final = 1
        try:
            client_q[key] = final
        except UnboundLocalError:
            print("Connection Refused")
            pass
    print('Final return_dict: ', return_dict)
    print('id_dict: ', id_dict)
    return client_q, id_dict


def ca_server(myip, return_dict, id_dict, num_neighbors, ips_sectable):
    with contextlib.suppress(OSError):
        ca = CA(random.randint(1000, 64000))
    proc = multiprocessing.Process(target=ca.as_server, args=(myip, return_dict, id_dict, num_neighbors, ips_sectable), daemon=True)
    proc.start()
    sleep(random.randint(1, 10))
    return proc


def update_table_ca(df, result, myID):
    '''
    function to update the mutual authentication table with the CA result.
    Note: this function should be called after the CA result is received.
    Note2: the new table is not being saved only converted to json and sent to the neighbors.
    '''
    #myID = int(list(set(df.loc[df['IP'] == co.get_ip_address(MESHINT), "ID"]))[0])
    print(result)
    df = df.assign(CA_Result=0)

    for res_tuple in result:
        res, cliIDs = res_tuple
        for ip in res.keys():
            if ip in df['IP'].values:
                if res[ip] in ['pass', 1]:
                    df.loc[df['IP'] == ip, 'CA_Result'] = 1
                elif res[ip] == 'fail' or res[ip] == 0:
                    df.loc[df['IP'] == ip, 'CA_Result'] = 2
                else:
                    df.loc[df['IP'] == ip, 'CA_Result'] = 3
                df.loc[df['IP'] == ip, 'CA_Server'] = myID
            else:
                try:
                    command = ['batctl', 't', ip] # Get mac of given ip
                    output = subprocess.run(command, shell=False, capture_output=True, text=True)
                    mac = output.stdout.replace('\n', '')
                except Exception as e:
                    mac = '----'
                client_mesh_name = ip.replace('.', '_')
                client_fpr, _ = pri.hashSig(f'pubKeys/{client_mesh_name}.der')
                if res[ip] in ['pass', 1]:
                    CA_Result = 1
                elif res[ip] == 'fail' or res[ip] == 0:
                    CA_Result = 2
                else:
                    CA_Result = 3
                # ip is used as ID (Need to check)
                try:
                    cliID = cliIDs[ip]
                except KeyError:
                    cliID = ip
                info = {'ID': cliID, 'MAC': mac, 'IP': ip, 'PubKey_fpr': client_fpr, 'MA_level': 1, 'CA_Result': CA_Result,
                        'CA_Server': myID} # Check if MA_level should be 1 or something else
                df = df.append(info, ignore_index=True)

    df.drop_duplicates(inplace=True)
    print('result: ', result)
    print('df: ', df)

    return df
