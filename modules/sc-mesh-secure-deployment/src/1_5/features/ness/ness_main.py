from pyke import knowledge_engine, krb_traceback
import sys
import pickle
import os

file_path = os.path.dirname(__file__)


class NESS:

    def __init__(self):
        self.engine = knowledge_engine.engine((__file__, '.compiled_krb'))
        self.dataset = f'{file_path}/input-simulator/output.data'

    def create_status_list(self, sec_list, n):
        return [sec_list[i][len(sec_list[i]) - 1] for i in range(n)]

    def create_good_server_list(self, sec_list, n):
        out = []
        for i in range(n):
            l = len(sec_list[i])
            if sec_list[i][l - 1] == 1:
                out.append(i)
        return out

    def mapping(self, nodes):
        mapp = dict(zip(nodes, range(len(nodes))))
        print("Map node labels with ID :", mapp)
        return mapp

    def remapping(self, mapps, position):
        # list out keys and values separately
        key_list = list(mapps.keys())
        val_list = list(mapps.values())
        return key_list[position]


    def create_servers_flags_list(self, sec_list, n, p):
        return [sec_list[i][p] for i in range(n)]

    def first_table(self, df, laststatus=None):
        '''
        Assuming that timestamp is small enough between rows
        '''
        nodes = df['IP'].to_list()
        latest_status_list = laststatus or [1 for _ in range(df.shape[0])]
        good_server_status_list = list(range(len(nodes)))
        flags_list = df["CA_Result"].tolist()
        mapp = self.mapping(df["ID"].tolist())
        servers_list = list(mapp.values())
        n = df.shape[0]

        return latest_status_list, good_server_status_list, flags_list, servers_list, n, mapp

    def adapt_table(self, result):
        latest_status_list = []
        good_server_status_list = []
        flags_list = []
        servers_list = []
        for node in result:
            servers_list.append(node)
            if result[node] == '65':
                latest_status_list.append(1)
                good_server_status_list.append(node)
                flags_list.append(1)
            if result[node] in ['131', '132', '133', '134']:
                latest_status_list.append(2)
                flags_list.append(2)
            if result[node] == '194':
                latest_status_list.append(3)
                flags_list.append(3)
        n = len(servers_list)
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
            mnode = ''
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
        mnode = i[0]
        print("\nDone")

        return action_code, mnode

    def read_file(self):
        with open(self.dataset, 'rb') as f:  # b for binary
            obj = pickle.load(f)
        return obj, len(obj)

    def test(self):
        T_struct, n = self.read_file()

        n_list = [n]
        sec_table_valid = 1

        init_latest_status_list = self.create_status_list(T_struct, n)
        init_latest_status_list1 = [init_latest_status_list]
        init_good_server_status_list = self.create_good_server_list(T_struct, n)
        p = 1
        init_servers_table = self.create_servers_flags_list(T_struct, n, p)
        p = 2
        init_flags_table = self.create_servers_flags_list(T_struct, n, p)

        self.run(init_latest_status_list, init_good_server_status_list, init_flags_table, init_servers_table, n)

    def run(self, init_latest_status_list, init_good_server_status_list, init_flags_table, init_servers_table, n):
        result = {}
        print("Running Decision on all nodes as the Sec Table is valid")
        print("Nodes status table: ", init_latest_status_list)
        print("Known good Servers table: ", init_good_server_status_list)
        print("All Servers table: ", init_servers_table)
        print("All Flags table: ", init_flags_table)
        n_list = [n]
        for i in range(n):
            #
            # Marshalling layer:
            # index of node to check
            #
            i_list = [i]
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

            act_code, mnode = self.run_decision(latest_status_list, good_server_status_list, flags_list, servers_list, nt, it)
            print("\nAction Code issued is ", act_code)
            print("\nfor node ", mnode)
            result[mnode] = act_code
        return result



