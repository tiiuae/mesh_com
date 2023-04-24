from flask import Flask, render_template
import json
import pandas as pd
import time
import itertools
import os
import subprocess

app = Flask(__name__)
debug = False

def read_topology():
    json_file = 'topology.json'

    try:
        with open(json_file) as f:
            topology = json.load(f)
        return topology
    except json.JSONDecodeError:
        print(f"Error: {json_file} does not contain valid JSON data.")
        return None

def get_neighbor_macs():
    macs = []
    proc = subprocess.Popen(['batctl', 'n'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    for i, x in enumerate(proc.stdout, start=1):
        if i > 2:
            aux = x.split()
            macs.append(aux[1].decode("utf-8"))
    return macs

def get_my_mac_address(interface='wlp1s0'):
    output = subprocess.check_output(['ifconfig', interface])
    for line in output.split(b'\n'):
        if b'ether ' in line:
            mac_address = line.split()[1]
            return mac_address.decode('utf-8')
    return None

def get_topology_info():
    '''
    :return df: Data frame with unique node and ness result
    :return nodes: List of {id, label, color} for each node
    :return edges: List of {from, to, label} for each edge
    '''
    nodes = []
    points = []
    edges = []

    file_dir = os.path.dirname(__file__)
    decision_table_file = str(file_dir).split('Demo')[0] + 'auth/decision_table.csv'
    if os.path.isfile(decision_table_file):
        df_decision = pd.read_csv(decision_table_file)
        if debug:
            print("Decision Table:")
            print(df_decision)
        df = df_decision.drop(columns=["PubKey_fpr", "MA_level", "CA_Result", "CA_Server"])
        df.drop_duplicates(inplace=True)
        if debug:
            print("Unique nodes in decision table:")
            print(df)

        for index, row in df.iterrows():
            node = {"id": row['MAC'], "label": row['MAC'], "color": 'green' if row['Ness_Result'] == 65 else 'red'}
            nodes.append(node)
            points.append(row['MAC'])

    else:  # If file is not present, create empty dataframe with column names, get neighbor macs and display yellow nodes
        df_decision = pd.DataFrame(
            columns=['ID', 'MAC', 'IP', 'PubKey_fpr', 'MA_level', 'CA_Result', 'CA_Server', 'Ness_Result'])
        if debug:
            print("Decision Table:")
            print(df_decision)
        df = df_decision.drop(columns=["PubKey_fpr", "MA_level", "CA_Result", "CA_Server"])
        df.drop_duplicates(inplace=True)
        if debug:
            print("Unique nodes in decision table:")
            print(df)

        macs = list(set([get_my_mac_address()] + get_neighbor_macs()))
        for mac in macs:
            node = {"id": mac, "label": mac, "color": 'yellow'}
            nodes.append(node)
            points.append(mac)

    # Get combinations of pair of 2 points from list of all nodes
    point_combinations = list(itertools.combinations(points, 2))

    # Generate list of edges
    for tuple in point_combinations:
        edge = {"from": tuple[0], "to": tuple[1], "label": "mesh"}
        edges.append(edge)

    return df, nodes, edges


@app.route('/')
def index():
    # Define variables for the topology
    df, nodes, edges = get_topology_info()

    if debug:
        print("Nodes: ", nodes)
        print("Edges: ", edges)

    data = {"nodes": nodes, "edges": edges}

    topology = data
    highlighted_row_index = 99999
    for co in range(len(topology["nodes"])):
        color = topology["nodes"][co]["color"]
        if  color != "green":
            highlighted_row_index = co


    # Render the HTML template and pass in the topology data as arguments
    return render_template('index.html', nodes=topology['nodes'], edges=topology["edges"],  data=df, highlighted_row_index=highlighted_row_index)

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True, use_reloader=True)