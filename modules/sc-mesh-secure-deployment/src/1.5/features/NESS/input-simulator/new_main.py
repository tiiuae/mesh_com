import argparse
import pickle
import random
import sys
from random import choices
import itertools
import PySimpleGUI as sg
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from pyke import knowledge_engine, krb_traceback
from pyvis.network import Network
import streamlit as st
import pandas as pd
import glob
import streamlit.components.v1 as components

# Construct the argument parser
# ap = argparse.ArgumentParser()
#
# # Add the arguments to the parser
# ap.add_argument("-d", "--dataset", required=True, help="Dataset Topology")
# args = ap.parse_args()

engine = knowledge_engine.engine(__file__)


def read_file(file):
    return nx.read_gml(file)


def plot(G):
    pos = nx.spring_layout(G)
    nx.draw_networkx_nodes(G, pos, cmap=plt.get_cmap('jet'), node_size=500)
    nx.draw_networkx_labels(G, pos)
    nx.draw_networkx_edges(G, pos, arrows=False)
    plt.interactive(False)
    plt.show(block=False)


def get_neighbors(G):
    neighs = {node: [neig for neig in nx.neighbors(G, node)] for node in G.nodes()}
    print('Neighbors: ')
    print(neighs)
    return neighs


def mapping(G):
    # mapp = dict(zip(range(len(G.nodes())), G.nodes()))
    mapp = dict(zip(G.nodes(), range(len(G.nodes()))))
    print("Map node labels with ID ")
    print(mapp)
    return nx.relabel_nodes(G, mapp)


def create_flags(G, node, mal, uncertain):
    flags = []
    population = [1, 3]  # possible flags
    weight = [0.99, 0.01]  # 3 should have a very low probability
    if node not in mal:
        for _ in range(len(list(nx.neighbors(G, node)))):
            flags.append(choices(population, weight)[0])
    else:
        for _ in range(len(list(nx.neighbors(G, node)))):
            flags.append(2)
    if node in uncertain:
        flags.append(-1)
    return flags


def create_cases(sel, num):
    ol = []
    pop = [0, 1, 2, 3, 4]
    w = [0.80, 0.15, 0.03, 0.01, 0.01]
    pop1 = [0, 1, 2, 3, 4]
    w1 = [0.80, 0.14, 0.04, 0.01, 0.01]
    if sel == 0:
        for _ in range(num):
            ol.append(choices(pop, w)[0])
    else:
        for _ in range(num):
            ol.append(choices(pop1, w1)[0])
    return ol


def gets_status(flags):
    voting = np.unique(flags, return_counts=True)
    if flags.count(1) == flags.count(2) and -1 not in voting[0]:  # if equal occurrences of 1 and 2 then results is 3
        return 3
    if len(voting[0]) > 1:
        return voting[0][np.argmax(voting[1])]
    else:
        return voting[0][0]


def get_malicious(G, banl, num_mal):
    mal = []
    for _ in range(num_mal):
        node = random.choice(list(G.nodes()))
        edges = list(G.edges(node))
        good_mnode = 0
        for edge in range(len(edges)):
            if edges[edge][1] in mal:
                good_mnode = 1
                break
        while node in mal or node in banl or good_mnode == 1:
            node = random.choice(list(G.nodes()))
            edges = list(G.edges(node))
            good_mnode = 0
            for edge in range(len(edges)):
                if edges[edge][1] in mal:
                    good_mnode = 1
                    break
        mal.append(node)
    print('Malicious Nodes: ', mal)

    return mal


def uncertain_node(G, malicious, banl, num_disc):
    uncer = []
    for _ in range(num_disc):
        node = random.choice(list(G.nodes()))
        edges = list(G.edges(node))
        good_unode = 0
        for edge in range(len(edges)):
            if edges[edge][1] in malicious:
                good_unode = 1
                break
        while node in malicious or node in uncer or node in banl or good_unode == 1:
            node = random.choice(list(G.nodes()))
            edges = list(G.edges(node))
            good_unode = 0
            for edge in range(len(edges)):
                if edges[edge][1] in malicious:
                    good_unode = 1
                    break
        uncer.append(node)
        edges = list(G.edges(node))
        for edge in range(len(edges)):
            G.remove_edge(edges[edge][0], edges[edge][1])
    print('Uncertain Nodes: ', uncer)

    return uncer


def disconnect_banned_node(node, G):
    edges = list(G.edges(node))
    res = -1
    for edge in range(len(edges)):
        G.remove_edge(edges[edge][0], edges[edge][1])
        print("\tNode: ", node, ". Disconnecting from node ", edges[edge][1])
        node1 = edges[edge][1]
        edges1 = list(G.edges(node1))
        if len(edges1) == 0:
            print("\tNode: ", node, ". Previously attached node ", node1,
                  "has no more edges and won't be used anymore.")
            res = node1

    return res


def do_CA(G, bl, num_mal, num_unc):
    output1 = []
    mal = get_malicious(G, bl, num_mal)
    uncertain = uncertain_node(G, mal, bl, num_unc)
    for node in G.nodes():
        if node not in bl:
            aux = []
            aux.append(node)
            aux.append(list(G.neighbors(node)))
            flags = create_flags(G, node, mal, uncertain)
            aux.append(flags)
            status = gets_status(flags)
            aux.append(status)
            output1.append(aux)
    return output1


def bw_CA_eval(nodes_list):
    bw_cnt = 0
    table_bit_cnt = 0
    st = len(nodes_list)
    if st < 16:
        infosize = 4
    else:
        infosize = 6
    for node in range(len(nodes_list)):
        aux = nodes_list[node]
        ls = len(aux[1])
        if ls == 0:
            ls = 1
        lf = len(aux[2])
        # each contributor is worth 4 bytes
        bw_cnt = bw_cnt + (8 * (ls + lf) + 4)
        table_bit_cnt = table_bit_cnt + infosize * ls + 2 * lf + 20

    bw_cnt = bw_cnt + int(table_bit_cnt / 8)

    return bw_cnt


def create_status_list(sec_list, n, original_n):
    out = [0] * original_n
    for i in range(n):
        l = len(sec_list[i])
        out[sec_list[i][0]] = sec_list[i][l - 1]
    return out


def create_good_server_list(sec_list, n):
    out = []
    for i in range(n):
        l = len(sec_list[i])
        if sec_list[i][l - 1] == 1:
            out.append(sec_list[i][0])
    return out


def create_servers_flags_list(sec_list, n, p):
    out = []
    for i in range(n):
        out.append(sec_list[i][p])
    return out


def run_decision(latest_status_list, good_server_status_list, flags_list, servers_list, n, i):
    engine.reset()

    engine.assert_('ness_fact', 'is_latest_status_list', ('latest_status_list', latest_status_list))
    engine.assert_('ness_fact', 'is_server_status_list', ('good_server_status_list', good_server_status_list))
    engine.assert_('ness_fact', 'is_flag_list', ('flags_list', flags_list))
    engine.assert_('ness_fact', 'is_server_list', ('servers_list', servers_list))
    engine.assert_('ness_fact', 'is_index', ('index', i))
    engine.assert_('ness_fact', 'is_number_nodes', ('number_nodes', n))
    # print("\nAdded facts:")
    # engine.get_kb('ness_fact').dump_specific_facts()

    engine.activate('ness_check')
    print("\nInferring for Good or Uncertain status...")
    res = 0
    try:
        with engine.prove_goal('ness_check.trust_analysis($eval)') as gen:
            for vars, plan in gen:
                if vars['eval'] == "Good":
                    act = "No signaling"
                    res = 1
                    action_code = 1 + 64
                else:
                    if vars['eval'] == "Uncertain":
                        act = "Signaling Uncertain Status"
                        res = 1
                        action_code = 2 + 128
                    else:
                        if vars['eval'] == "NotChecked":
                            act = "Signaling Not Checked Status"
                            res = 1
                            action_code = 3 + 128
    except Exception1:
        krb_traceback.print_exc()
        sys.exit(1)

    if res == 1:
        print("\nAction is: ", act, "for node ", i[0])
    else:
        res1 = 0
        print("\nCan't conclude inference. More checks needed for node ", i[0])
        try:
            with engine.prove_goal('ness_check.consistency_analysis($eval1)') as gen:
                for vars, plan in gen:
                    if vars['eval1'] == "Good":
                        res1 = 1
                    else:
                        if vars['eval1'] == "NotChecked":
                            res1 = 2
        except Exception2:
            krb_traceback.print_exc()
            sys.exit(1)

        if res1 == 0:
            act = "Signaling Security Table data Consistency or Servers trust Issues"
            print("Action is: ", act, "on node ", i[0])
            action_code = 4 + 128
        else:
            if res1 == 2:
                act = "Signaling Security Table data Consistency for Not Checked case"
                print("Action is: ", act, "on node ", i[0])
                action_code = 5 + 128
            else:
                act = "Signaling Suspected Malicious"
                print("Action is: ", act, "node ", i[0])
                action_code = 6 + 128 + 64

    print("\nDecision Reported!")

    return action_code


def trust_eval(sec_analysis_results, n, G):
    eval_res = []
    ban = []
    trigger_event = 0
    malic = 6 + 128 + 64
    print("\nTrusting Analysis on going...\n")

    for i in range(n):
        cur_entry = sec_analysis_results[i]
        action = cur_entry[1]
        if action == malic:
            ban.append(cur_entry[0])
            print("Trusting - Banning node:", cur_entry[0])
            node = cur_entry[0]
            node_val = disconnect_banned_node(node, G)
            if node_val >= 0:
                ban.append(node_val)
        else:
            if trigger_event == 0 and action > 128:
                trigger_event = 1
                print("Trusting - Triggering CA event due to node", cur_entry[0])

    if trigger_event == 0:
        print("Trusting - No specific event. Proceeding to next scheduled CA")
    eval_res.append(ban)
    eval_res.append(trigger_event)

    return (eval_res)


def set_ui(simtime, ui_report_list, new_topo):
    shape = ui_report_list.shape
    net = Network(height='90%', width='100%', heading='Simulation ' + str(simtime) + 'ms')
    net.from_nx(new_topo)
    # net.show("example0.html")
    sg.change_look_and_feel('Dark Blue 3')
    layout = [[sg.Text('Nodes Status View', font='Courier 14')], [sg.Text(' ', font='Courier 11')],
              [sg.Text('Status code:', font='Courier 11')], [sg.Text('0 : Banned Node', font='Courier 11')],
              [sg.Text('2 : Trusted Node', font='Courier 11')], [sg.Text('4 : Uncertain Node', font='Courier 11')],
              [sg.Text('5 : Unchecked Node', font='Courier 11')],
              [sg.Text('6 : Server trust issue with Node', font='Courier 11')],
              [sg.Text('7 : Inconsistent data for Unchecked Node', font='Courier 11')],
              [sg.Text('9 : Suspected Malicious Node', font='Courier 11')], [sg.Text(' ', font='Courier 11')],
              [sg.Text(size=(5, 1), key='-LAZY0-', font='Courier 11'), sg.Text('ms', font='Courier 11')],
              [sg.Text(size=(60, 1), key='-LAZYC-', font='Courier 11')]]
    for i in range(shape[0]):
        layout.append([sg.Text(size=(60, 1), key='-LAZY' + str(i + 1) + '-', font='Courier 11')])
    layout.append([sg.Text(size=(60, 1), key='-LAZYS-', font='Courier 11')])
    layout.append([sg.Text(' ', font='Courier 11')])
    layout.append([sg.Button('Exit'), sg.Button('Cont')])

    return sg.Window('Security Status', layout, finalize=True)


def run_ui(window, ui_report_list, simtime, status_str='No Signaling', sim_status='Simulation Started'):
    shape = ui_report_list.shape
    while True:  # Event Loop
        event, values = window.read(timeout=1000)  # Time in Milliseconds before returning
        if event in (None, 'Cont'):
            break
        window['-LAZY0-'].update(str(simtime))
        window['-LAZYC-'].update(status_str)
        s = '\xA0'
        x = 0
        for i in range(shape[0]):
            window['-LAZY' + str(i + 1) + '-'].update(
                s.join(['n' + str(x + j) + ':' + str(int(ui_report_list[i][j])) for j in range(shape[1])]))
            x += shape[1]
        window['-LAZYS-'].update(sim_status)
        if event in (None, 'Exit'):
            window['-LAZYS-'].update("Simulation Ended")
            window.close()
            quit()
            break


def update_ui_st(simtime, dataf, init_latest_status_list, ui_level, security_res, ui_report_list):
    # net = Network(height='90%', width='100%', heading='Simulation ' + str(simtime) + 'ms')
    net = Network(height='100%', width='100%', notebook=True)  # , heading='Simulation ' + str(simtime) + 'ms')
    net.from_nx(new_topo)
    ui_report_list_line = []
    #  if (btn):
    dataf.dataframe(ui_report_list)
    # net.show('example0.html')
    HtmlFile = open("example0.html", 'r', encoding='utf-8')
    source_code = HtmlFile.read()
    components.html(source_code, height=900, width=900)
    linecnt = 0
    for j in range(initial_n):
        if j == ui_report_list.shape[1]:
            linecnt = linecnt + 1
        if init_latest_status_list[j] == 0:
            ui_report_list_line.append(0)
        else:
            for k in range(len(ui_level)):
                if ui_level[k][0] == j:
                    ui_report_list_line.append(ui_level[k][1])
    aux = np.array_split(ui_report_list_line, ui_report_list.shape[0])
    for index in range(ui_report_list.shape[0]):
        ui_report_list[index] = aux[index]
    if security_res[1] == 0:
        status_str = "No Signaling"
    else:
        status_str = "Triggering CA"
        if len(security_res[0]) > 0:
            status_str = status_str + " - Banning nodes: "
            for strl in range(len(security_res[0])):
                status_str = status_str + str(security_res[0][strl]) + "  "


# run_ui(window, ui_report_list, simtime, status_str, 'Simulation Running')

def step_simulation(initial_n, ui_report_list, new_topo):
    dataf = st.empty()
    #
    # This is where the loop on CA events should start
    #
    iteration_cnt = 0
    iteration_cnt1 = 1
    simtime = 0
    itcnt = 0
    total_bw_cnt = 0
    total_node_seq_cnt = 0
    simexcept = 0
    n = initial_n
    banned_nodes = []
    nattackmin = 2
    ncases = 20
    while True:
        mal_nb_list = create_cases(0, ncases)
        tcount = mal_nb_list.count(1) + mal_nb_list.count(2) + mal_nb_list.count(3) + mal_nb_list.count(4)
        if tcount > nattackmin:
            break
    print("Stats of CA events - malicious nodes:\t", mal_nb_list)
    st.text("Stats of CA events - malicious nodes:\t" + str(mal_nb_list))
    while True:
        unc_nb_list = create_cases(1, ncases)
        tcount = unc_nb_list.count(1) + unc_nb_list.count(2) + unc_nb_list.count(3) + unc_nb_list.count(4)
        if tcount > nattackmin:
            break
    print("Stats of CA events - unconnected nodes: ", unc_nb_list)
    st.text("Stats of CA events - unconnected nodes: " + str(unc_nb_list))

    while simtime < 20000 and simexcept == 0:
        print("\n#############################")
        print("# Simulation time = ", simtime, "ms")
        st.text("# Simulation time = " + str(simtime) + " ms")
        print("#############################\n")
        # val = input("Enter number of malicious nodes: ")
        val = mal_nb_list[iteration_cnt]
        # val2 = input("Enter number of uncertain/disconnected nodes: ")
        val2 = unc_nb_list[iteration_cnt1]
        print("Current number of nodes = ", n)
        if (val + val2 + 4) > n:
            simexcept = 1
            print("\nNot enough nodes remaining in the network! Stopping Simulation...")
            break
        new_topo1 = new_topo.copy()
        print("Generating CA events:")
        output = do_CA(new_topo1, banned_nodes, int(val), int(val2))
        inst_bw_cnt = bw_CA_eval(output)
        total_bw_cnt = total_bw_cnt + inst_bw_cnt
        print('Current Security table as provided by nodes from CA process: \n')
        if simtime == 0:
            print('[node_num, [svr1, svr2, ..., svrk], [flg1, flg2, ..., flgk], status]]\n')
        print(output)
        with open('output.data', 'wb') as filehandle:
            # store the data as binary data stream
            pickle.dump(output, filehandle, pickle.HIGHEST_PROTOCOL)
        T_struct = output
        sec_analysis_results = []
        ui_level = []
        n_list = [n]
        sec_table_valid = 1
        init_latest_status_list = create_status_list(T_struct, n, initial_n)
        init_latest_status_list1 = []
        init_latest_status_list1.append(init_latest_status_list)
        init_good_server_status_list = create_good_server_list(T_struct, n)
        p = 1
        init_servers_table = create_servers_flags_list(T_struct, n, p)
        p = 2
        init_flags_table = create_servers_flags_list(T_struct, n, p)
        #
        # Marshalling to prepare dashboard and Decision Function call if table valid
        #
        if sec_table_valid == 1:
            print("\nRunning Decision on all nodes as the Sec Table is valid\n")
            print("Nodes status table: ", init_latest_status_list)
            print("Known good Servers table: ", init_good_server_status_list)
            print("All Servers table: ", init_servers_table)
            print("All Flags table: ", init_flags_table)
            for i in range(n):
                #
                # Marshalling layer:
                # index of node to check
                #
                i_list = []
                node_entry = T_struct[i]
                i_list.append(node_entry[0])
                #
                # Preparing tuples for the query into Pyke engine
                #
                latest_status_list = tuple(init_latest_status_list)
                good_server_status_list = tuple(init_good_server_status_list)
                flags_list = tuple(init_flags_table[i])
                servers_list = tuple(init_servers_table[i])
                nt = tuple(n_list)
                it = tuple(i_list)
                act_code = run_decision(latest_status_list, good_server_status_list, flags_list, servers_list, nt, it)
                print("\nAction Code issued is ", act_code)
                node_res = []
                ui_level_elem = []
                node_res.append(node_entry[0])
                ui_level_elem.append(node_entry[0])
                node_res.append(act_code)
                ui_level_elem.append((int(act_code / 64) + (act_code % 64)))
                sec_analysis_results.append(node_res)
                ui_level.append(ui_level_elem)
            sec_table_valid = 0

        security_res = trust_eval(sec_analysis_results, n, new_topo)
        n_correct = len(security_res[0])
        ban_list = security_res[0]
        for j in range(len(ban_list)):
            banned_nodes.append(ban_list[j])
        if security_res[1] == 1:
            simtime = simtime + 100
            print("\nFull instant BW consumed during this round (broadcast included) =", (10 * (inst_bw_cnt + 8)),
                  "bytes/s")
        else:
            simtime = simtime + 1000
            print("\nFull instant BW consumed during this round (broadcast included) =", (inst_bw_cnt + 8),
                  "bytes/s")

        sec_table_valid = 1
        old_n = n
        n = n - n_correct
        iteration_cnt = (iteration_cnt + 7) % ncases
        iteration_cnt1 = (iteration_cnt1 + 3) % ncases
        itcnt = itcnt + 1
        total_node_seq_cnt = total_node_seq_cnt + old_n
        #
        # Updating Report windows
        #

        update_ui_st(simtime, dataf, init_latest_status_list, ui_level, security_res, ui_report_list)

    print("\nTotal simulated time =", simtime, "ms, and number of iterations that were run =", itcnt)
    print("\nAverage BW consumed during simulation =", int(1000 * total_bw_cnt / simtime), "bytes/s")
    print("Average number of nodes per iteration =", int(total_node_seq_cnt / itcnt))
    print("Average BW consumed per node =", int(1000 * total_bw_cnt * itcnt / (simtime * total_node_seq_cnt)),
          "bytes/s")
    print("\nSIMULATION END")

    st.text("\nTotal simulated time =" + str(simtime) + " ms, and number of iterations that were run =" + str(itcnt))
    st.text("\nAverage BW consumed during simulation = " + str(int(1000 * total_bw_cnt / simtime)) + "bytes/s")
    st.text("Average number of nodes per iteration = " + str(int(total_node_seq_cnt / itcnt)))
    st.text("Average BW consumed per node = " + str(
        int(1000 * total_bw_cnt * itcnt / (simtime * total_node_seq_cnt))) + " bytes/s")
    st.text("\nSIMULATION END")


def get_matrix(n):  # get matrix for any input (to print values)
    aux = 0
    for i in range(2, 10):
        if n % i == 0:
            aux = i
            size = int(n / aux)

    matrix = np.zeros((int(size), aux))
    return matrix + 2


def load_data():
    files = glob.glob('datasets/*.gml')
    option = st.selectbox('Select Dataset: ', files)

    return option


def stream_ui():
    st.sidebar.title('Nodes Status View')
    st.sidebar.text('')
    st.sidebar.text('Status code:')
    st.sidebar.text('2 : Trusted Node')
    st.sidebar.text('5 : Unchecked Node')
    st.sidebar.text('6 : Server trust issue with Node')
    st.sidebar.text('7 : Inconsistent data for Unchecked Node')
    st.sidebar.text('9 : Suspected Malicious Node')
    st.sidebar.text('')
    st.sidebar.image('images/TII.png')


if __name__ == '__main__':
    st.title("SSRC SecComm Simulator")

    # topo = read_file('datasets/geant2012.gml')  # open file
    # topo = read_file(args.dataset)  # open file
    dataset = load_data()
    topo = read_file(dataset)  # open file
    initial_n = len(topo.nodes)

    get_neighbors(topo)  # get the name neighbors (only to print)
    new_topo = mapping(topo)  # map names with IDs
    neighbors = get_neighbors(new_topo)  # now getting the real neighbors

    ui_report_list = get_matrix(initial_n)
    btn = st.button('Start Simulation')
    stream_ui()
    # window = set_ui(0, ui_report_list, new_topo)
    # run_ui(window, ui_report_list, 0)
    if btn:
        step_simulation(initial_n, ui_report_list, new_topo)
