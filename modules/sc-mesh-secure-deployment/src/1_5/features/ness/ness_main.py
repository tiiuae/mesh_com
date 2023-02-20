from pyke import knowledge_engine, krb_traceback
import sys
import pickle
import os
import json
import time
import numpy as np
import pandas as pd
from .simulator import main
from common import utils
import traceback

file_path = os.path.dirname(__file__)

class NodeType:
    BENIGN = 65
    SUSPICIOUS = 194
    MALICIOUS = 2 #from CA
    UNCERTAIN = 131
    NOT_CHECKED = 132
    INCONSISTENT = 133

class NESS:

    def __init__(self):
        self.engine = knowledge_engine.engine((__file__, '.compiled_krb'))

    def create_status_list(self, sec_list, n):
        return [sec_list[i][len(sec_list[i]) - 1] for i in range(n)]

    def mapping(self, nodes):
        mapp = dict(zip(nodes, range(len(nodes))))
        print("Map node labels with ID :", mapp)
        return mapp

    def remapping(self, mapps, position):
        # list out keys and values separately
        key_list = list(mapps.keys())
        val_list = list(mapps.values())
        return key_list[position]

    def create_good_server_list(self, sec_list, n):
        out = []
        for i in range(n):
            l = len(sec_list[i])
            #if sec_list[i][l - 1] == 1: # commenting this, let's assume all are benign
            out.append(i)
        return out

    def create_servers_flags_list(self, sec_list, n, p):
        out = []
        for i in range(n):
            out.append(sec_list[i][p])
        return out

    def save_file_status(self, **args):
        aux = {arg: args[arg] for arg in args}
        aux['timestamp'] = time.time()
        jsonStr = json.dumps(aux)
        with open("last_result.json", "w") as outfile:
            outfile.write(jsonStr)

    def get_table(self, df):
        flags_map = {65: 1, 131: 2, 132: 2, 133: 2, 134: 2, 194: 3}  # needs to add disconection (-1)
        mappids = self.mapping(set(df["ID"].tolist()))
        df2 = df.replace({"CA_Server": mappids})
        # df2 = df2.replace({"ID": mapp})
        df2["CA_Server"] = df2["CA_Server"].astype('Int64')
        new_list = []
        last_status = []
        for i in df2["ID"]:
            df2.loc[(df2['CA_Server'].isnull()) & (df2["ID"] == i), "CA_Server"] = int(list(pd.Series(i).map(mappids))[0])
            table = [list(pd.Series(i).map(mappids))[0]]
            servers = list(df2.loc[df2['ID'] == i, "CA_Server"])
            table.append(servers)
            flags = list(df2.loc[df2['ID'] == i, "CA_Result"])
            table.append(flags)
            count = np.unique(flags, return_counts=True)
            if len(count[1]) > 1:
                final = count[0][0] if count[1][0] > count[1][1] else count[0][1]
            else:
                final = count[0][0]
            table.append(final)
            new_list.append(table)
        finaltable = []
        for l in new_list:
            if l not in finaltable:
                finaltable.append(l)
        return sorted(finaltable, key=lambda x: x[0]), mappids

    def adapt_table(self, result, mapp):
        if not os.path.isfile('last_result.json'):
            latest_status_list = []
            good_server_status_list = []
            flags_list = []
            servers_list = []
        else:
            f = open('last_result.json')
            json_object = json.load(f)
            latest_status_list = json_object['latest_status_list']
            good_server_status_list = json_object['good_server_status_list']
            flags_list = json_object['flags_list']
            servers_list = json_object['servers_list']
            mapp = json_object['mapp']
        for node in result:
            if result[node] == 65:
                latest_status_list.append(1)
                good_server_status_list.append(node)
                flags_list.append(1)
            if result[node] in [131, 132, 133, 134]:
                latest_status_list.append(2)
                flags_list.append(2)
            if result[node] == 194:
                latest_status_list.append(3)
                flags_list.append(3)
        n = len(servers_list)
        self.save_file_status(latest_status_list=latest_status_list, good_server_status_list=good_server_status_list,
                              flags_list=flags_list, servers_list=servers_list, mapp=mapp)

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
        """
        unit test should be run as
        ness = ness_main.NESS()
        ness.test()
        stores log to logs/ness-log.txt
        """
        common_ut = utils.Utils()
        logger = common_ut.setup_logger('ness')

        self.dataset = f'{file_path}/simulator/output.data'
        if not os.path.isfile(self.dataset):
            sim=main.Simulator()
            sim.run()

        output = self.read_file()
        logger.debug("Data:\n%s", output[0])
        try:
            result = self.run_all(output[0])
            logger.info("Decision engine executed")
            logger.debug("Result:\n%s", result)
        except Exception as e:
            traceback.print_exc()
            logger.error("Decision engine failed with exception %s", e)

        common_ut.close_logger(logger)

    def ness_result_to_table(self, df, ness_result, mapp):
        df = df.assign(Ness_Result=0)
        for res in ness_result:
            ind = self.remapping(mapp, res)
            df.loc[df['ID'] == ind, 'Ness_Result'] = int(ness_result[res])
        df.drop_duplicates(inplace=True)
        return df

    def run_all(self, output, latest_status=False):
        result = {}
        T_struct = output
        n = len(T_struct)
        n_list = [n]
        sec_table_valid = 1
        init_latest_status_list = self.create_status_list(T_struct, n)
        init_latest_status_list1 = []
        init_latest_status_list1.append(init_latest_status_list)
        init_good_server_status_list = self.create_good_server_list(T_struct, n)
        p = 1
        init_servers_table = self.create_servers_flags_list(T_struct, n, p)
        p = 2
        init_flags_table = self.create_servers_flags_list(T_struct, n, p)
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
                # if latest_status:
                #     f = open('last_result.json')
                #     json_object = json.load(f)
                #     latest_status_list = json_object['latest_status_list']
                #     good_server_status_list = json_object['good_server_status_list']
                #     flags_list = json_object['flags_list']
                #     servers_list = json_object['servers_list']
                #     mapp = json_object['mapp']
                # else:
                latest_status_list = tuple(init_latest_status_list)
                good_server_status_list = tuple(init_good_server_status_list)
                flags_list = tuple(init_flags_table[i])
                servers_list = tuple(init_servers_table[i])
                nt = tuple(n_list)
                it = tuple(i_list)

                act_code, nnode = self.run_decision(latest_status_list, good_server_status_list, flags_list,
                                                    servers_list, nt,
                                                    it)
                print("\nFor node: ", nnode)
                print("Action Code issued is ", act_code)

                result[nnode] = act_code
        return result

    def run_all_new(self, output):
        '''
        Notchecked  status not implemented
        Uncertain  status not implemented on CA
        '''
        result = {}
        for node in output:
            if len(node) != 4:
                result[node[0]] = NodeType.INCONSISTENT
                continue
            node_id, num_servers, flags, ca_result = node
            if len(flags) != len(num_servers):
                result[node_id] = NodeType.INCONSISTENT
                continue
            count_malicious = flags.count(NodeType.MALICIOUS)
            if count_malicious == len(num_servers):
                result[node_id] = NodeType.MALICIOUS
            elif count_malicious > len(num_servers) / 2:
                result[node_id] = NodeType.SUSPICIOUS
            else:
                result[node_id] = NodeType.BENIGN
        return result