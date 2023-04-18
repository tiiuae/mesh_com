import itertools
import os
from  PIL import Image

import pandas as pd


import streamlit as st
import streamlit.components.v1 as components
import networkx as nx
from pyvis.network import Network
import subprocess

from streamlit_autorefresh import st_autorefresh

file_dir = os.path.dirname(__file__)

# Autorefresh every 5 seconds
st_autorefresh(interval=5000, key="new-ui")

#Add a logo (optional) in the sidebar
logo = Image.open(str(file_dir) +'/ssrc_logo.png')
st.sidebar.image(logo)

image = Image.open(str(file_dir) +'/mesh_shield_logo.png')
st.image(image, width=600)

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

def get_id_from_mac(mac):
    cmd = ["echo", "-n", mac]
    cmd2 = ["b2sum", "-l", "32"]
    with subprocess.Popen(cmd, stdout=subprocess.PIPE) as ps_proc:
        with subprocess.Popen(cmd2, stdin=ps_proc.stdout, stdout=subprocess.PIPE) as grep_ipython_proc:
            pid, _ = grep_ipython_proc.communicate()
    id = pid.decode().split('\n', maxsplit=1)[0].split('  -', maxsplit=1)[0]
    return id

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
        df = df_decision.drop(columns=["PubKey_fpr", "MA_level", "CA_Result", "CA_Server"])
        df.drop_duplicates(inplace=True)

        for index, row in df.iterrows():
            node = {"id": row['ID'], "label": row['ID'], "color": 'green' if row['Ness_Result'] == 65 else 'red'}
            nodes.append(node)
            points.append(row['ID'])

    else:  # If file is not present, create empty dataframe with column names, get neighbor macs and display yellow nodes
        df_decision = pd.DataFrame(
            columns=['ID', 'MAC', 'IP', 'PubKey_fpr', 'MA_level', 'CA_Result', 'CA_Server', 'Ness_Result'])
        df = df_decision.drop(columns=["PubKey_fpr", "MA_level", "CA_Result", "CA_Server"])
        df.drop_duplicates(inplace=True)

        macs = list(set([get_my_mac_address()] + get_neighbor_macs()))
        for mac in macs:
            id = get_id_from_mac(mac)
            node = {"id": id, "label": id, "color": 'orange'}
            nodes.append(node)
            points.append(id)

    # Get combinations of pair of 2 points from list of all nodes
    point_combinations = list(itertools.combinations(points, 2))

    # Generate list of edges
    for tuple in point_combinations:
        edge = {"from": tuple[0], "to": tuple[1], "label": "mesh"}
        edges.append(edge)

    return df, nodes, edges


def color_coding(row):
    return ['font-size: 100px; background-color:pink'] * len(
        row) if row.Ness_Result == 194 else ['font-size: 100px; background-color:white'] * len(row)




def main():
    # st.markdown(""" <style> .font {
    # font-size:50px ; font-family: 'Courier'; color: #808080;}
    # </style> """, unsafe_allow_html=True)
    # st.markdown('<p class="font">Mesh Shield 1.5</p>', unsafe_allow_html=True)
    # Define variables for the topology
    df, nodes, edges = get_topology_info()

    # style
    th_props = [
        ('font-size', '18px'),
        ('text-align', 'center'),
        ('font-weight', 'bold'),
        ('color', '#6d6d6d'),
        ('background-color', '#D3D3D3')
    ]

    td_props = [
        ('text-align', 'center'),
        ('font-size', '18px')
    ]

    styles = [
        dict(selector="th", props=th_props),
        dict(selector="td", props=td_props)
    ]

    # table
    df2 = df.style.apply(color_coding, axis=1).set_properties(**{'text-align': 'center'}).set_table_styles(styles)

    # Create an empty graph
    G = nx.Graph()

    # Add nodes to the graph
    for node in nodes:
        G.add_node(node['id'], label=node['label'], color=node['color'], size=20,font='16px arial black')

    # Add edges to the graph
    for edge in edges:
        G.add_edge(edge['from'], edge['to'], label=edge['label'],length=200,font='15px arial black')

    st.graphviz_chart(
        f"""
        graph {{
            {"; ".join([f"{node['id']} [label={node['label']}, color={node['color']}, style=filled]" for node in nodes])};
            {"; ".join([f"{edge['from']} -- {edge['to']} [label={edge['label']}]" for edge in edges])};
        }}
        """
    )


    st.title("Decision Table")
    #st.write(df)
    st.table(df2)

    st.title("Topology Information")
    #net = Network(height='100%', width='100%', notebook=True)  # , heading='Simulation ' + str(simtime) + 'ms')
    net = Network(height="300px", width="100%", notebook=True)
    net.from_nx(G)
    net.show('example.html')
    HtmlFile = open("example.html", 'r', encoding='utf-8')
    from IPython.core.display import display, HTML
    display(HTML('example.html'), figsize=(10,5))
    source_code = HtmlFile.read()
    components.html(source_code, height=1200, width=1000)
if __name__ == "__main__":
    main()