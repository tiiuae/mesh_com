
import networkx as nx
import numpy as np
#import matplotlib.pyplot as plt
import random
from random import choices
import pickle
import argparse
import os
file_path = os.path.dirname(__file__)


# Construct the argument parser
ap = argparse.ArgumentParser()

# Add the arguments to the parser
ap.add_argument("-d", "--dataset", required=False, help="Dataset topology")
ap.add_argument("-i", "--input", required=False, help="Running as a demo", default=False)
args = ap.parse_args()

class Simulator:

    def __init__(self):
        if not args.dataset:
            self.topo = self.read_file(f'{file_path}/datasets/geant2012.gml')
        else:
            self.topo = self.read_file(args.dataset)  # open file
        self.get_neighbors(self.topo)  # get the name neighbors (only to print)
        self.new_topo = self.mapping(self.topo)  # map names with IDs
        self.neighbors = self.get_neighbors(self.new_topo)  # now getting the real neighbors
        if args.input:
            self.malicious = input("Enter number of malicious nodes: ")
            self.disconnected = input("Enter number of uncertain/disconnected nodes: ")
        else:
            self.malicious = random.randint(1, int(self.topo.number_of_nodes() / 2))  # malicious nodes
            self.disconnected = random.randint(1, int(self.topo.number_of_nodes() / 3))  # disconnected nodes

    def read_file(self, file):
        return nx.read_gml(file)


    # def plot(self, G):
    #     pos = nx.spring_layout(G)
    #     nx.draw_networkx_nodes(G, pos, cmap=plt.get_cmap('jet'), node_size=500)
    #     nx.draw_networkx_labels(G, pos)
    #     nx.draw_networkx_edges(G, pos, arrows=False)
    #     plt.show(block=False)


    def get_neighbors(self, G):
        neighs = {node: [neig for neig in nx.neighbors(G, node)] for node in G.nodes()}
        print('Neighbors: ')
        print(neighs)
        return neighs


    def mapping(self, G):
        # mapp = dict(zip(range(len(G.nodes())), G.nodes()))
        mapp = dict(zip(self.topo.nodes(), range(len(self.topo.nodes()))))
        print("Map node labels with ID ")
        print(mapp)
        return nx.relabel_nodes(G, mapp)


    def create_flags(self, G, node, mal, uncertain):
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


    def gets_status(self, flags):
        voting = np.unique(flags, return_counts=True)
        if flags.count(1) == flags.count(2) and -1 not in voting[0]:  # if equal number of occurrence between 1 and 2 then results is 3
            return 3
        if len(voting[0]) > 1:
            return voting[0][np.argmax(voting[1])]
        else:
            return voting[0][0]


    def get_malicious(self, G, num_mal):
        mal = []
        for _ in range(num_mal):
            node = random.choice(list(G.nodes()))
            mal.append(node)
        print('Malicious Nodes: ', mal)
        return mal


    def uncertain_node(self, G, malicious, num_disc):
        uncer = []
        for _ in range(num_disc):
            node = random.choice(list(G.nodes()))
            if node not in malicious:
                uncer.append(node)
                edges = list(G.edges(node))
                for edge in range(len(edges)):
                    G.remove_edge(edges[edge][0], edges[edge][1])
        print('Uncertain Nodes: ', uncer)
        return uncer


    def create_tuple(self, G, num_mal, num_unc):
        output = []
        mal = self.get_malicious(G, num_mal)
        uncertain = self.uncertain_node(G, mal, num_unc)
        for node in G.nodes():
            aux = []
            aux.append(node)
            aux.append(list(G.neighbors(node)))
            flags = self.create_flags(G, node, mal, uncertain)
            aux.append(flags)
            status = self.gets_status(flags)
            aux.append(status)
            output.append(aux)
        return output

    def save_file(self, output):
        path_file = f'{file_path}/output.data'
        print(path_file)
        with open(path_file, 'wb') as filehandle:
            # store the data as binary data stream
            pickle.dump(output, filehandle, pickle.HIGHEST_PROTOCOL)

    def run(self):
        output = self.create_tuple(self.new_topo, int(self.malicious), int(self.disconnected))
        print('Final List: \n')
        print('[node_num, [svr1, svr2, ..., svrk], [flg1, flg2, ..., flgk], status]]\n')
        print(output)
        self.save_file(output)


if __name__ == '__main__':
    sim = Simulator()
    sim.run()




