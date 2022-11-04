from header import *


def continuous_authentication(sectable):
    print("Starting Continuous Authentication")
    loop_ca = asyncio.get_event_loop()
    ca_task = loop_ca.create_task(ca_utils.launchCA(sectable))
    with contextlib.suppress(asyncio.CancelledError):
        result = loop_ca.run_until_complete(asyncio.gather(ca_task))
    if result[0] is not None:
        sectable = ca_utils.update_table_ca(sectable, result)
        return sectable
    else:
        return None


def only_ca():
    try:
        table = 'auth/dev.csv'
        filetable = pd.read_csv(table)
        filetable.drop_duplicates(inplace=True)
        if not filetable.empty:
            sectable = continuous_authentication(filetable)
            if sectable is not None:
                received = ut.exchage_table(sectable)
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


def listeningMBA():
    q = queue.Queue()
    mal = mba.MBA(mesh_utils.get_mesh_ip_address())
    threading.Thread(target=mal.client, args=(q,)).start()
    return q.get()


def announcing(message):
    mal = mba.MBA(mesh_utils.get_mesh_ip_address())
    threading.Thread(target=mal.server, args=(message, True,), daemon=True).start()


def decision_engine(sectable, ma, q):
    print("Starting Decision Engine")
    table_file = 'auth/dev.csv'
    output, mapp = ness.get_table(sectable)
    if "Ness_Result" not in sectable.columns:
        threading.Thread(target=ma.client, args=(q,), daemon=True).start()
        ness_result = ness.run_all(output)
    else:
        ness_result = ness.run_all(output, True)
    ness.adapt_table(ness_result, mapp)
    sectable = ness.ness_result_to_table(sectable, ness_result, mapp)
    sectable.to_csv(table_file, mode='w', header=True, index=False)
    print("Decision Engine Done")
    return ness_result, mapp


def quaran(ness_result, q, sectable, ma, mapp):
    for node in ness_result:
        if ness_result[node] == 194 or (not q.empty() and q.get() == 'malicious'):
            print("Malicious Node: ", ness.remapping(mapp, node))
            mac = sectable.iloc[node]['IP']
            threading.Thread(target=ma.server, args=(f"malicious: {mac}", True), daemon=True).start()
            threading.Thread(target=qua.block, args=(mac,), daemon=True).start()


def sec_beat(myID):
    q = queue.Queue()
    ut.checkiptables()
    ma = mba.MBA(mesh_utils.get_mesh_ip_address())
    sectable = only_ca(myID)
    ness_result, first_round, mapp = decision_engine(sectable, ma, q)
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
    return mut.myID


def start_servers():
    process = []
    q = queue.Queue()
    ca_s = ca_utils.ca_server(mesh_utils.get_mesh_ip_address())  # port 9999
    process.append(ca_s)
    ex_s = multiprocessing.Process(target=ut.exchange_server, args=(q,), daemon=True)  # port 5005
    process.append(ex_s)
    return process


if __name__ == "__main__":
    sec_beat_time = 5
    # Mutual Authentication
    myID = mutual_authentication()
    ness_result = {}
    start_servers()
    while True:
        if mesh_utils.verify_mesh_status():  # verifying that mesh is running
            sleep(sec_beat_time - time() % sec_beat_time)  # sec beat time
            first_round, ness_result = sec_beat(ness_result, myID)
        else:
            print("No mesh established")
