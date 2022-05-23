# driver.py

import contextlib
import sys
import matplotlib.pyplot as plt
from pyke import knowledge_engine, krb_traceback
import pickle

engine = knowledge_engine.engine(__file__)


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
    print("\nAdded fact:")
    engine.get_kb('ness_fact').dump_specific_facts()

    engine.activate('ness_check')
    #	print('open is assigned to %r' % open)
    print("\nInferring for Good or Uncertain status...")
    res = 0
    try:
        with engine.prove_goal('ness_check.trust_analysis($eval)') as gen:
            for vars, plan in gen:
                if vars['eval'] == "Good":
                    act = "No signaling"
                    res = 1
                    action_code = 1 + 64
                elif vars['eval'] == "Uncertain":
                    act = "Signaling Uncertain Status"
                    res = 1
                    action_code = 3 + 128
    except Exception:
        krb_traceback.print_exc()
        sys.exit(1)

    if (res == 1):
        print("\nAction is: ", act, "for node ", i[0])
    else:
        res1 = 0
        print("\nCan't conclude inferrence. More checks needed for node ", i[0])
        try:
            with engine.prove_goal('ness_check.consistency_analysis($eval1)') as gen:
                for vars, plan in gen:
                    if vars['eval1'] == "Good":
                        res1 = 1
        except Exception:
            krb_traceback.print_exc()
            sys.exit(1)

        if res1 == 0:
            act = "Signaling Security Table data Consistency or Servers trust Issues"
            print("Action is: ", act, "on node ", i[0])
            action_code = 4 + 128
        else:
            act = "Signaling Suspected Malicious"
            print("Action is: ", act, "node ", i[0])
            action_code = 2 + 128 + 64

    print("\nDone")

    return action_code


def read_file(file):
    with open(file, 'rb') as f:  # b for binary
        obj = pickle.load(f)
    return obj, len(obj)
    #


# Security list entry, total number of nodes and validation flag
# each entry is a node: [node_num, [svr1, svr2, ..., svrk], [flg1, flg2, ..., flgk], status]]
#
# T_struct = [[0, [1], [1], 1], [1, [0, 2], [1, 1], 1], [2, [0, 1], [1, 3], 3], [3, [2, 1], [1], 1],
#             [4, [0, 1], [2, 2], 2]]
# n = 5


T_struct, n = read_file('input-simulator/output.data')

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

fig, ax = plt.subplots()
ax.set_axis_off()

colLabels = []

for name in range(n):
    colLabels.append('N' + str(name))
table = ax.table(
    cellText=init_latest_status_list1,
    #	rowLabels = [],
    # colLabels=['Node0', 'Node1', 'Node2', 'Node3', 'Node4'],
    colLabels=colLabels,
    #	rowColours =["palegreen"] * 5,
    #	colColours =["palegreen"] * 5,
    cellLoc='center',
    loc='upper left')
table.auto_set_font_size(False)

table.set_fontsize(25)

ax.set_title('Derived Nodes Security Status Table', fontsize=30)
plt.show(block=False)

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
        # flags_list = tuple(init_flags_table[i])
        # servers_list = tuple(init_servers_table[i])
        flags_list = tuple(init_flags_table)
        servers_list = tuple(init_servers_table)
        nt = tuple(n_list)
        it = tuple(i_list)

        act_code = run_decision(latest_status_list, good_server_status_list, flags_list, servers_list, nt, it)
        print("\nAction Code issued is ", act_code)

plt.show()
