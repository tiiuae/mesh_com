import itertools
import os
from  PIL import Image

import pandas as pd


import streamlit as st
import streamlit.components.v1 as components
import networkx as nx
from pyvis.network import Network

file_dir = os.path.dirname(__file__)

#Add a logo (optional) in the sidebar
logo = Image.open(str(file_dir).split('Demo')[0]+'/ssrc_logo.png')
st.sidebar.image(logo)

image = Image.open(str(file_dir).split('Demo')[0]+'/mesh_shield_logo.png')
st.image(image)


def get_topology_info(df_decision):
    '''
    :param df_decision: Data frame from decision table
    :return df: Data frame with unique node and ness result
    :return nodes: List of {id, label, color} for each node
    :return edges: List of {from, to, label} for each edge
    '''
    df = df_decision.drop(columns=["PubKey_fpr", "MA_level", "CA_Result", "CA_Server"])
    df.drop_duplicates(inplace=True)

    nodes = []
    points = []
    edges = []

    for index in range(len(df)):
        node = {"id": df['MAC'].iloc[index], "label": df['MAC'].loc[index],
                "color": 'green' if df['Ness_Result'].iloc[index] == 65 else 'red'}
        nodes.append(node)
        points.append(df['MAC'].iloc[index])

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
    file_dir = os.path.dirname(__file__)
    decision_table_file = str(file_dir).split('Demo')[0] + 'auth/decision_table.csv'
    if os.path.isfile(decision_table_file):
        df_decision = pd.read_csv(decision_table_file)
    else: # If file is not present, create empty dataframe with column names
        df_decision = pd.DataFrame(columns=['ID', 'MAC', 'IP', 'PubKey_fpr', 'MA_level','CA_Result','CA_Server','Ness_Result'])

    df, nodes, edges = get_topology_info(df_decision)

    # style
    th_props = [
        ('font-size', '25px'),
        ('text-align', 'center'),
        ('font-weight', 'bold'),
        ('color', '#6d6d6d'),
        ('background-color', '#D3D3D3')
    ]

    td_props = [
        ('text-align', 'center'),
        ('font-size', '25px')
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
        G.add_node(node['id'], label=node['label'], color=node['color'], size=25,font='20px arial black')

    # Add edges to the graph
    for edge in edges:
        G.add_edge(edge['from'], edge['to'], label=edge['label'],font='20px arial black')

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
    net = Network(height='100%', width='100%', notebook=True)  # , heading='Simulation ' + str(simtime) + 'ms')
    net.from_nx(G)
    net.show('example.html')
    HtmlFile = open("example.html", 'r', encoding='utf-8')
    from IPython.core.display import display, HTML
    display(HTML('example.html'), figsize=(10,5))
    source_code = HtmlFile.read()
    components.html(source_code, height=1200, width=1000)

if __name__ == "__main__":
    main()