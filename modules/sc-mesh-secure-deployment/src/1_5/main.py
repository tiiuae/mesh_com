from header import *
from main_with_menu import *

def continuous_authentication(sectable, myID):
    print("Starting Continuous Authentication")
    aux = sectable.iloc[:, :5]
    sectable = aux.drop_duplicates()
    sectable.drop(sectable.loc[sectable['MAC'] == '----'].index, inplace=True) # To avoid duplicates after exchange table for mesh neighbors not originally mutually authenticated
    # loop_ca = asyncio.get_event_loop()
    try:
        loop_ca = asyncio.get_event_loop()
    except RuntimeError as ex:
        if "There is no current event loop in thread" in str(ex):
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop_ca = asyncio.get_event_loop()
    ca_task = loop_ca.create_task(ca_utils.launchCA(sectable))
    with contextlib.suppress(asyncio.CancelledError):
        result = loop_ca.run_until_complete(asyncio.gather(ca_task))
    if result[0] is not None:
        sectable = ca_utils.update_table_ca(sectable, result, myID)
        return sectable
    else:
        return None


def only_ca(myID):
    try:
        table = 'auth/dev.csv'
        filetable = pd.read_csv(table)
        filetable.drop_duplicates(inplace=True)
        if not filetable.empty:
            if "Ness_Result" in filetable.columns:
                filetable.drop(["Ness_Result"], axis=1, inplace=True)
            sectable = continuous_authentication(filetable, myID)
            sleep(2)
            if sectable is not None:
                # ut.exchage_table(sectable)
                sleep(2)
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
    table_file = 'auth/decision_table.csv'
    output, mapp = ness.get_table(sectable)
    threading.Thread(target=ma.client, args=(q,), daemon=True).start()
    ness_result = ness.run_all_new(output)
    # if "Ness_Result" not in sectable.columns:
    #     threading.Thread(target=ma.client, args=(q,), daemon=True).start()
    #     ness_result = ness.run_all(output)
    # else:
    #     ness_result = ness.run_all(output, True)
    ness.adapt_table(ness_result, mapp)
    sectable = ness.ness_result_to_table(sectable, ness_result, mapp)
    sectable.to_csv(table_file, mode='w', header=True, index=False)
    print("Security Table with Decision")
    information = pd.read_csv("common/information.csv")
    print(information)
    print("\n\n")
    print(sectable)
    print("\nDecision Engine Done")
    return ness_result, mapp


def quaran(ness_result, q, sectable, ma, mapp):
    quarantineTime = 20  # seconds
    for node in ness_result:
        if ness_result[node] == 194 or (not q.empty() and q.get() == 'malicious'):
            print("Malicious Node: ", ness.remapping(mapp, node))
            #mac = sectable.iloc[node]['IP']
            mac = sectable[sectable['ID'] == ness.remapping(mapp, node)]['IP'].unique()[0]
            threading.Thread(target=ma.server, args=(f"malicious: {mac}", True), daemon=True).start()
            threading.Thread(target=qua.block, args=(mac, quarantineTime), daemon=True).start()
        else:
            print("Nothing to do")

def sec_beat(myID):
    q = queue.Queue()
    ut.checkiptables()
    ma = mba.MBA(mesh_utils.get_mesh_ip_address())
    # Start exchange table server so that it can receive messages as soon as other nodes complete cont auth
    start_server_thread = ut.start_server()
    sectable = only_ca(myID)
    sectable.drop_duplicates(inplace=True)
    ut.exchage_table(sectable, start_server_thread)
    global_table = pd.read_csv('auth/global_table.csv')
    ness_result, mapp = decision_engine(global_table, ma, q)
    #quaran(ness_result, q, sectable, ma, mapp)
    quaran(ness_result, q, global_table, ma, mapp)

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
    #ex_s = multiprocessing.Process(target=ut.exchange_server, args=(q,), daemon=True)  # port 5005
    #process.append(ex_s)
    return process

MA_thread = None
def readfile():
    with open("features.yaml", "r") as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)


def initialize(feature):
    global MA_thread
    if feature == 'mutual':
            MA_thread = MA()
    if feature == 'continuous':
            CA()
    if feature == 'NESS':
            DE()
    if feature == 'secbeat':
            sbeat_client()
    if feature == 'quarantine':
            Quarantine()
    if feature == 'only_mesh':
            only_mesh()

if __name__ == "__main__":
    threadList = []
    features = readfile()
    for index in features:
        if features[index]:
            initialize(index)
    # wait for Auth_AP to start in background for future nodes
    if MA_thread:
        MA_thread.join()