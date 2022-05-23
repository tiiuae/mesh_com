import contextlib
import sys
from pyke import knowledge_engine, krb_traceback
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import random
from random import choices
import pickle
import argparse

# Construct the argument parser
ap = argparse.ArgumentParser()

# Add the arguments to the parser
ap.add_argument("-d", "--dataset", required=True, help="Dataset topology")
args = ap.parse_args()

engine = knowledge_engine.engine(__file__)


def read_file(file):
    return nx.read_gml(file)


def plot(G):
    pos = nx.spring_layout(G)
    nx.draw_networkx_nodes(G, pos, cmap=plt.get_cmap('jet'), node_size=500)
    nx.draw_networkx_labels(G, pos)
    nx.draw_networkx_edges(G, pos, arrows=False)
    plt.show(block=False)


def get_neighbors(G):
    neighs = {node: [neig for neig in nx.neighbors(G, node)] for node in G.nodes()}
    print('Neighbors: ')
    print(neighs)
    return neighs


def mapping(G):
    # mapp = dict(zip(range(len(G.nodes())), G.nodes()))
    mapp = dict(zip(topo.nodes(), range(len(topo.nodes()))))
    print("Map node labels with ID ")
    print(mapp)
    return nx.relabel_nodes(G, mapp)


def create_flags(G, node, mal, uncertain):
    flags = []
    if node not in mal:
        population = [1, 3]  # possible flags
        weight = [0.99, 0.01]  # 3 should have a very low probability
        for _ in range(len(list(nx.neighbors(G, node)))):
            flags.append(choices(population, weight)[0])
    else:
        for _ in range(len(list(nx.neighbors(G, node)))):
            flags.append(2)
    if node in uncertain:
        flags.append(-1)
    return flags


def gets_status(flags):
    voting = np.unique(flags, return_counts=True)
    if flags.count(1) == flags.count(2) and -1 not in voting[
        0]:  # if equal number of occurrence between 1 and 2 then results is 3
        return 3
    if len(voting[0]) > 1:
        return voting[0][np.argmax(voting[1])]
    else:
        return voting[0][0]


def get_malicious(G, num_mal):
    mal = []
    for _ in range(num_mal):
        node = random.choice(list(G.nodes()))
        mal.append(node)
    print('Malicious Nodes: ', mal)
    return mal


def uncertain_node(G, malicious, num_disc):
    uncer = []
    for _ in range(num_disc):
        node = random.choice(list(G.nodes()))
        if node not in malicious:
            uncer.append(node)
            edges = list(G.edges(node))
            for edge_ in edges:
                G.remove_edge(edge_[0], edge_[1])
    print('Uncertain Nodes: ', uncer)
    return uncer


def create_tuple(G, num_mal, num_unc):
    output1 = []
    mal = get_malicious(G, num_mal)
    uncertain = uncertain_node(G, mal, num_unc)
    for node in G.nodes():
        aux = [node, list(G.neighbors(node))]
        flags = create_flags(G, node, mal, uncertain)
        aux.append(flags)
        status = gets_status(flags)
        aux.append(status)
        output1.append(aux)
    return output1


def create_status_list(sec_list, n):
    out = []
    for i in range(n):
        l = len(sec_list[i])
        out.append(sec_list[i][l - 1])
    return out


def create_good_server_list(sec_list, n):
    out = []
    for i in range(n):
        l = len(sec_list[i])
        if sec_list[i][l - 1] == 1:
            out.append(i)
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
    print("\nAdded facts:")
    engine.get_kb('ness_fact').dump_specific_facts()

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
                        action_code = 3 + 128
                    else:
                        if vars['eval'] == "NotChecked":
                            act = "Signaling Not Checked Status"
                            res = 1
                            action_code = 4 + 128
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
            action_code = 5 + 128
        else:
            if res1 == 2:
                act = "Signaling Security Table data Consistency for Not Checked case"
                print("Action is: ", act, "on node ", i[0])
                action_code = 6 + 128
            else:
                act = "Signaling Suspected Malicious"
                print("Action is: ", act, "node ", i[0])
                action_code = 2 + 128 + 64

    print("\nDone")

    return action_code


def run_all(output):
    T_struct = output
    n = len(T_struct)
    n_list = []
    n_list.append(n)
    sec_table_valid = 1
    init_latest_status_list = create_status_list(T_struct, n)
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
        print("Running Decision on all nodes as the Sec Table is valid")
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
            i_list.append(i)
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


if __name__ == '__main__':
    # topo = read_file('datasets/geant2012.gml')  # open file
    topo = read_file(args.dataset)  # open file
    get_neighbors(topo)  # get the name neighbors (only to print)
    new_topo = mapping(topo)  # map names with IDs
    neighbors = get_neighbors(new_topo)  # now getting the real neighbors
    val = input("Enter number of malicious nodes: ")
    val2 = input("Enter number of uncertain/disconnected nodes: ")
    output = create_tuple(new_topo, int(val), int(val2))
    print('Final List: \n')
    print('[node_num, [svr1, svr2, ..., svrk], [flg1, flg2, ..., flgk], status]]\n')
    print(output)
    run_all(output)
    with open('output.data', 'wb') as filehandle:
        # store the data as binary data stream
        pickle.dump(output, filehandle, pickle.HIGHEST_PROTOCOL)
