from header import *

async def launchCA(ID, sectable, mod):
    """
    this function should start server within localhost and client within the neighbors
    It should be the trigger in time-bases ex: every X seconds
    """
    with contextlib.suppress(OSError):
        ca = ca_main.CA(ID)
    myip = co.get_ip_address(MESHINT)
    max_count = 3
    flag_ctr = 0
    proc = multiprocessing.Process(target=ca.as_server, args=(myip,))
    proc.start()
    sleep(random.randint(1, 10))
    if not mod:
        neigh = mesh_utils.get_macs_neighbors()
        for ne in neigh:
            if ne not in sectable["MAC"].values:  # means that this is neighbor but was authenticated by someone else
                ne_ips = mesh_utils.get_neighbors_ip()
                for ip in ne_ips:
                    if ip not in sectable["IP"].values:
                        info = {'ID': '---', 'MAC': ne, 'IP': ip,
                                'PubKey_fpr': "___", 'MA_level': -1}
                        mut = mutual.Mutual(MUTUALINT)
                        mut.update_table(info)
            sectable = pd.read_csv('aux/dev.csv')
            ind = sectable.index[sectable['MAC'] == ne].tolist()[0]
            IP = sectable.loc[ind]["IP"]
            if IP != myip:
                flag_ctr = ca_client(IP, flag_ctr, max_count, ca)
    else:  # modular version MAC are not the ones in neighbors but are the bat0
        neigh = mesh_utils.get_arp()
        for IP in neigh:
            if IP != myip:
                flag_ctr = ca_client(IP, flag_ctr, max_count, ca)
    proc.terminate()
    proc.join()
    return client_q


def ca_client(IP, flag_ctr, max_count, ca):
    print("neighbor IP:", IP)
    manager = multiprocessing.Manager()
    return_dict = manager.dict()
    p = multiprocessing.Process(target=ca.as_client, args=(IP, return_dict), daemon=True)
    jobs = [p]
    p.start()
    for proc in jobs:
        proc.join()
    sleep(5)
    for key in return_dict.keys():
        count = np.unique(return_dict[key], return_counts=True)
        if len(count[1]) > 1:
            final = count[0][0] if count[1][0] > count[1][1] else count[0][1]
        else:
            final = count[0][0]
    try:
        client_q[IP] = final
    except UnboundLocalError:
        print("Connection Refused")
        pass
    flag_ctr += 1
    if flag_ctr == max_count:
        p.terminate()
        # break
    return flag_ctr


def update_table_ca(df, result):
    """
    function to update the mutual authentication table with the CA result.
    Note: this function should be called after the CA result is received.
    Note2: the new table is not being saved only converted to json and sent to the neighbors.
    """
    df = df.assign(CA_Result=0)
    for index, row in df.iterrows():
        for res in result:
            for ip in res.keys():
                if row['IP'] == ip:
                    IP = row['IP']
                    if res[ip] in ['pass', 1]:
                        df.loc[index, 'CA_Result'] = 1
                    elif res[ip] == 'fail':
                        df.loc[df['IP'] == IP, 'CA_Result'] = 2
                    else:
                        df.loc[df['IP'] == IP, 'CA_Result'] = 3
                    df.loc[df['IP'] == IP, 'CA_ts'] = time()
    df.drop_duplicates(inplace=True)
    return df


def continuous_authentication(sectable, mod):
    print("Starting Continuous Authentication")
    loop_ca = asyncio.get_event_loop()
    ca_task = loop_ca.create_task(launchCA(random.randint(1000, 64000), sectable, mod))
    with contextlib.suppress(asyncio.CancelledError):
        result = loop_ca.run_until_complete(asyncio.gather(ca_task))
    if result[0] is not None:
        sectable = update_table_ca(sectable, result)
        return sectable
    else:
        return None


def only_ca(mod=False):
    try:
        table = 'auth/dev.csv'
        filetable = pd.read_csv(table)
        filetable.drop_duplicates(inplace=True)
        if not filetable.empty:
            sectable = continuous_authentication(filetable, mod)
            if sectable is not None:
                received = exchage_table(sectable, mod)
                for ind in received:
                    new = pd.read_json(received[ind].decode())
                    sectable = pd.concat([sectable, new], ignore_index=True)
                sectable.drop_duplicates(inplace=True)
                if 'CA_Result' in set(sectable):
                    sectable.to_csv(table, index=False)
                else:
                    sectable.to_csv(table, mode='a', header=False, index=False)
                return sectable
            print("End of Continuous Authentication")
        else:
            print("Empty Security Table")
            exit()
    except FileNotFoundError:
        print("SecTable not available. Need to be requested during provisioning")


def exchage_table(df, mod):
    '''
    this function exchange the table (json) with the neighbors
    '''
    q = queue.Queue()
    client_q = {}
    message = df.to_json()
    if not mod:
        neigh = mesh_utils.get_macs_neighbors()
        for ne in neigh:
            ind = df.index[df['MAC'] == ne].tolist()[0]
            IP = df.loc[ind]["IP"]
            print(IP)
            client_q = exchange_process(IP, message, client_q, q)
    else:  # modular version MAC are not the ones in neighbors but are the bat0
        neigh = mesh_utils.get_arp()
        for IP in neigh:
            client_q = exchange_process(IP, message, client_q, q)
    return client_q


def exchange_process(IP, message, client_q, q):
    proc = threading.Thread(target=server, args=(IP, message,))  # lock variable add later
    proc.daemon = True
    proc.start()
    p = threading.Thread(target=client, args=(q,), daemon=True)
    jobs = [p]
    p.start()
    client_q[IP] = q.get()
    print(client_q[IP])
    for proc in jobs:
        proc.join()
    return client_q


def client(q, debug=False):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("0.0.0.0", 5005))
    data, addr = sock.recvfrom(1024)
    print("Message received")
    if debug:
        print(data, addr)
    q.put(data)


def server(IP, message, debug=False):
    num_message = 5
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setblocking(False)
    while num_message:
        if debug:
            print(f"this is server and it will send message {num_message} more times")
        sock.sendto(bytes(message, "utf-8"), (IP, 5005))
        num_message -= 1
        sleep(1)


def listeningMBA():
    q = queue.Queue()
    mal = mba.MBA(mesh_utils.get_mesh_ip_address())
    threading.Thread(target=mal.client, args=(q,)).start()
    return q.get()


def announcing(message):
    mal = mba.MBA(mesh_utils.get_mesh_ip_address())
    threading.Thread(target=mal.server, args=(message, True,), daemon=True).start()


def checkiptables():
    '''
    this function check if any iptables was blocked before
    '''
    files = glob.glob('features/quarantine/iptables*')
    if files:
        data = 0
        for fi in files:
            da = fi.split('-')[1]
            if da > data:
                data = da
        path = f'features/quarantine/iptables-{str(data)}'
        command = ['iptables-restore', '<', path]
        subprocess.call(command, shell=False)


def decision_engine(first_round, ness_result, sectable, ma, q):
    print("Starting Decision Engine")
    if first_round:
        latest_status_list, good_server_status_list, flags_list, servers_list, nt, mapp = ness.first_table(sectable)
        threading.Thread(target=ma.client, args=(q,), daemon=True).start()
        first_round = False
    else:
        latest_status_list, good_server_status_list, flags_list, servers_list, nt, mapp = ness.adapt_table(ness_result)
    ness_result = ness.run(latest_status_list, good_server_status_list, flags_list, servers_list, nt)
    print("Decision Engine Done")
    return ness_result, first_round, mapp


def quarantine(ness_result, q, sectable, ma, mapp):
    for node in ness_result:
        if ness_result[node] == 194 or (not q.empty() and q.get() == 'malicious'):
            print("Malicious Node: ", ness.remapping(mapp, node))
            mac = sectable.iloc[node]['IP']
            threading.Thread(target=ma.server, args=("malicious: " + mac, True), daemon=True).start()
            threading.Thread(target=qua.block, args=(mac,), daemon=True).start()


def sec_beat(first_round, ness_result):
    q = queue.Queue()
    checkiptables()
    ma = mba.MBA(mesh_utils.get_mesh_ip_address())
    sectable = only_ca()
    ness_result, first_round, mapp = decision_engine(first_round, ness_result, sectable, ma, q)
    quarantine(ness_result, q, sectable, ma, mapp)
    return first_round, ness_result


def mutual_authentication():
    print("Starting Mutual Authentication")
    mut = mutual.Mutual(MUTUALINT)
    loop = asyncio.get_event_loop()
    mutual_task = loop.create_task(mut.start())
    with contextlib.suppress(asyncio.CancelledError):
        loop.run_until_complete(mutual_task)
    sleep(5)
    print("End of Mutual Authentication")


if __name__ == "__main__":
    sec_beat_time = 5
    first_round = True  # flag for the decision engine. If not information available, create a table.
    # Mutual Authentication
    mutual_authentication()
    ness_result = {}
    while True:
        if mesh_utils.verify_mesh_status():  # verifying that mesh is running
            sleep(sec_beat_time - time() % sec_beat_time)  # sec beat time
            first_round, ness_result = sec_beat(first_round, ness_result)
        else:
            print("No mesh established")
