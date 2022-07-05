from pyke import knowledge_engine, krb_traceback
import sys
import pickle
import os

file_path = os.path.dirname(__file__)


class NESS:

    def __init__(self):
        self.engine = knowledge_engine.engine((__file__, '.compiled_krb'))
        self.dataset = file_path + '/input-simulator/output.data'

    def create_status_list(self, sec_list, n):
        out = []

        for i in range(n):
            l = len(sec_list[i])
            out.append(sec_list[i][l - 1])

        return out

    def create_good_server_list(self, sec_list, n):
        out = []

        for i in range(n):
            l = len(sec_list[i])
            if sec_list[i][l - 1] == 1:
                out.append(i)

        return out

    def create_servers_flags_list(self, sec_list, n, p):
        out = []

        for i in range(n):
            out.append(sec_list[i][p])

        return out

    def adapt_table(self, df):
        '''
        Assuming that timestamp is small enough between rows
        '''
        nodes = df['IP'].to_list()
        latest_status_list = []
        good_server_status_list = tuple(range(len(nodes)))
        flags_list = df["CA_Result"].tolist()
        servers_list = df["ID"].tolist()
        n = df.shape[0]

        return latest_status_list, good_server_status_list, flags_list, servers_list, n

    def run_decision(self, latest_status_list, good_server_status_list, flags_list, servers_list, n, i):
        self.engine.reset()
        self.engine.assert_('ness_fact', 'is_latest_status_list', ('latest_status_list', latest_status_list))
        self.engine.assert_('ness_fact', 'is_server_status_list', ('good_server_status_list', good_server_status_list))
        self.engine.assert_('ness_fact', 'is_flag_list', ('flags_list', flags_list))
        self.engine.assert_('ness_fact', 'is_server_list', ('servers_list', servers_list))
        self.engine.assert_('ness_fact', 'is_index', ('index', i))
        self.engine.assert_('ness_fact', 'is_number_nodes', ('number_nodes', n))
        print("\nAdded fact:")
        self.engine.get_kb('ness_fact').dump_specific_facts()

        self.engine.activate('ness_check')
        #	print('open is assigned to %r' % open)
        print("\nInferring for Good or Uncertain status...")
        res = 0
        try:
            with self.engine.prove_goal('ness_check.trust_analysis($eval)') as gen:
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

        if res == 1:
            print("\nAction is: ", act, "for node ", i[0])
        else:
            res1 = 0
            print("\nCan't conclude inference. More checks needed for node ", i[0])
            try:
                with self.engine.prove_goal('ness_check.consistency_analysis($eval1)') as gen:
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

    def read_file(self):
        with open(self.dataset, 'rb') as f:  # b for binary
            obj = pickle.load(f)
        return obj, len(obj)

    def test(self):
        T_struct, n = self.read_file()

        n_list = []
        n_list.append(n)
        sec_table_valid = 1

        init_latest_status_list = self.create_status_list(T_struct, n)
        init_latest_status_list1 = []
        init_latest_status_list1.append(init_latest_status_list)
        init_good_server_status_list = self.create_good_server_list(T_struct, n)
        p = 1
        init_servers_table = self.create_servers_flags_list(T_struct, n, p)
        p = 2
        init_flags_table = self.create_servers_flags_list(T_struct, n, p)

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

            act_code = self.run_decision(latest_status_list, good_server_status_list, flags_list, servers_list, nt, it)
            print("\nAction Code issued is ", act_code)

    def run(self, init_latest_status_list, init_good_server_status_list, init_flags_table, init_servers_table, n):

        print("Running Decision on all nodes as the Sec Table is valid")
        print("Nodes status table: ", init_latest_status_list)
        print("Known good Servers table: ", init_good_server_status_list)
        print("All Servers table: ", init_servers_table)
        print("All Flags table: ", init_flags_table)
        n_list = []
        n_list.append(n)
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

            act_code = self.run_decision(latest_status_list, good_server_status_list, flags_list, servers_list, nt, it)
            print("\nAction Code issued is ", act_code)



