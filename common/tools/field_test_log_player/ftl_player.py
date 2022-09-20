#!/usr/bin/python3
"""
Plot static and moving node
"""
import getopt
import os
import random
import string
import sys
import time
from datetime import datetime

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import pandas as pd
import pylab
import utm
from matplotlib.pyplot import pause

LOOP_SECONDS = 1


# pylint: disable=too-many-instance-attributes
class Node:
    """
    Single node data collector
    """

    def __init__(self, _path, _filename, _mode):
        self.indoor_mode = _mode
        # imported data lists from csv log file
        self.__f_neighbour_list = []
        self.__f_originators_list = []
        self.__f_timestamps = []
        self.__f_gps_timestamps = []
        self.__f_lat_loc = []
        self.__f_lon_loc = []
        self.__f_rssi = []
        self.__f_noise = []
        self.__f_txmcs = []
        self.__f_txtp = []
        self.__f_rxtp = []

        self.my_mac = None  # node MAC address
        self.__matched_row_offset = 0  # row offset
        self.__matched_time_s = 0  # offset in seconds to the laziest one

        try:
            self.__load_data_from_file(_path, _filename)
        except Exception as error:
            print(f"Can't load values from: {_filename}\nException- {error}")
            sys.exit()

    def __load_data_from_file(self, __path, __filename):
        """
        Read data from CSV file and fill in internal variables
        :return:
        """
        timestamp_name = "Timestamp"
        neighbours_name = "Neighbors"
        originators_name = "Originators"
        noise_name = "noise [dBm]"
        lat_name = "latitude"
        gps_time = "GPS time"
        lon_name = "longitude"
        rssi_name = "rssi [MAC,dBm;MAC,dBm ...]"
        txmcs_name = "TX MCS [MAC,MCS;MAC,MCS ...]"
        rx_tp_name = "RX throughput [Bits/s]"
        tx_tp_name = "TX throughput [Bits/s]"

        self.my_mac = __filename.split("_")[2].split(".")[0]
        df = pd.read_csv(f"{__path}/{__filename}", engine='python', on_bad_lines='warn')

        df.drop(columns=["channel", "txpower [dBm]",
                         "RX MCS [MAC,MCS;MAC,MCS ...]",
                         "altitude", "PDOP",
                         "cpu temp [mC]", "wifi temp [mC]", "tmp100 [mC]",
                         "battery voltage [uV]", "battery current [uA]", "nRF voltage [mV]",
                         "nRF current [mA]", "3v3 voltage [mV]", "3v3 current [mA]"], inplace=True)

        # Drop corrupted DF lines i.e. the ones where the
        # last expected csv file column is empty.
        df.dropna(subset=[df.columns[-1]], inplace=True)

        # GPS information available if not indoor
        if self.indoor_mode is False:
            df.drop(df[df.latitude < -999].index, inplace=True)
            df.drop(df[df.latitude == 0].index, inplace=True)
            df[lat_name].replace('', np.nan, inplace=True)
            df.dropna(subset=[lat_name], inplace=True)
            df[lon_name].replace('', np.nan, inplace=True)
            df.dropna(subset=[lon_name], inplace=True)
            df[gps_time].replace('', np.nan, inplace=True)
            df.dropna(subset=[gps_time], inplace=True)
            df[gps_time] = pd.to_datetime(df[gps_time])
            # to seconds
            self.__f_gps_timestamps = [ns / 1000000000 for ns in df[gps_time].values.tolist()]
            self.__f_lat_loc = df[lat_name].values.tolist()
            self.__f_lon_loc = df[lon_name].values.tolist()
        else:
            # use system timestamps in indoor case
            df[timestamp_name].replace('', np.nan, inplace=True)
            df.dropna(subset=[timestamp_name], inplace=True)
            self.__f_timestamps = [
                time.mktime(datetime.strptime(str(ts), "%Y-%m-%d %H:%M:%S").timetuple()) for ts in
                df[timestamp_name].values.tolist()]

        # originators column not available in old field test logger files
        if originators_name in df.columns:
            self.__f_originators_list = df[originators_name].values.tolist()

        self.__f_neighbour_list = df[neighbours_name].values.tolist()
        self.__f_rssi = df[rssi_name].values.tolist()
        self.__f_noise = df[noise_name].values.tolist()
        self.__f_txmcs = df[txmcs_name].values.tolist()
        self.__f_rxtp = df[rx_tp_name].values.tolist()
        self.__f_txtp = df[tx_tp_name].values.tolist()

    def get_location_utm(self):
        """
        get location from synced time offset point
        :return:
        """
        easting, northing, _, _ = \
            utm.from_latlon(self.__f_lat_loc[self.__matched_row_offset],
                            self.__f_lon_loc[self.__matched_row_offset])
        return easting, northing

    def get_rssi(self):
        """
        get rssi from synced time offset point
        :return:
        """
        return self.__f_rssi[self.__matched_row_offset]

    def get_noise(self):
        """
        get noise from synced time offset point
        :return:
        """
        return self.__f_noise[self.__matched_row_offset]

    def get_txmcs(self):
        """
        get txmcs from synced time offset point
        :return:
        """
        return self.__f_txmcs[self.__matched_row_offset]

    def get_tx_throughput(self):
        """
        get TX TrhoughPut from synced time offset point
        :return:
        """
        return self.__f_txtp[self.__matched_row_offset]

    def get_rx_throughput(self):
        """
        get RX TrhoughPut from synced time offset point
        :return:
        """
        return self.__f_rxtp[self.__matched_row_offset]

    def get_originator(self):
        """
        get neighbours from synced time offset point
        :return:
        """
        return self.__f_originators_list[self.__matched_row_offset]

    def get_neighbours(self):
        """
        get neighbours from synced time offset point
        :return:
        """
        return self.__f_neighbour_list[self.__matched_row_offset]

    def get_mac(self):
        """
        get mac from synced time offset point
        :return:
        """
        return self.my_mac

    def get_time_stamp_in_s(self, row):
        """
        get gps timestamp_s from synced time offset point or
        in indoor case take from system time.
        :return:
        """
        if self.indoor_mode:
            return self.__f_timestamps[row]
        return self.__f_gps_timestamps[row]

    def update_row_offset_from_seconds_offset(self, seconds_offset):
        """
        called every update round to check drifting
        :param seconds_offset:
        :return:
        """

        if self.indoor_mode:
            timetable = self.__f_timestamps
        else:
            timetable = self.__f_gps_timestamps

        for index, n in enumerate(timetable):
            if n > self.__matched_time_s + seconds_offset:
                self.__matched_row_offset = index
                # print(index, n)
                return

    def set_time_offset(self, seconds):
        """
        Called in startup to calculate offset to log start
        :param seconds:
        :return:
        """
        start = self.get_time_stamp_in_s(0)
        matched = start + seconds

        if self.indoor_mode:
            timetable = self.__f_timestamps
        else:
            timetable = self.__f_gps_timestamps

        for index, _ in enumerate(timetable):
            if self.get_time_stamp_in_s(index) >= matched:
                self.__matched_row_offset = index
                self.__matched_time_s = self.get_time_stamp_in_s(index)
                print(f"  row offset {self.__matched_row_offset} lines")
                print(f"  matched time {self.__matched_time_s} seconds")
                return

        raise RuntimeError("not found")


# pylint: disable=too-many-instance-attributes
class NodeNetwork:
    """
    Plot static and moving nodes with matplotlib
    """

    def __init__(self, mode, _active_path, _rxtx):
        self.nx_edges_list = []
        self.nx_nodes_list = []
        self.edge_labels = {}
        self.nx_graph = nx.MultiDiGraph()
        self.nx_trace = nx.MultiDiGraph()
        self.nx_data_path = nx.MultiDiGraph()
        self.sec_offset_from_start = 0
        self.trace_this_mac = ""
        self.network_nodes = []
        self.__bbox = None  # outlines for drawing
        self.pause = False
        self.indoor_mode = mode
        self.rxtx = _rxtx
        if len(_active_path) > 0:
            self.active_path_macs = _active_path.split(",")[0], _active_path.split(",")[1]
        else:
            self.active_path_macs = ""
        self.active_path_list = []

    def __bbox_of_network(self):
        """
        Bounding Box with wgs-84 utm coordinates
        :param:
        :return:
        """
        bbox_offset = 10

        lon_min = 10000000
        lon_max = -10000000
        lat_min = 10000000
        lat_max = -10000000

        for node in self.network_nodes:
            lat, lon = node.get_location_utm()
            lat_max = lat if lat > lat_max else lat_max
            lat_min = lat if lat < lat_min else lat_min
            lon_max = lon if lon > lon_max else lon_max
            lon_min = lon if lon < lon_min else lon_min

        return (lon_min - bbox_offset,
                lon_max + bbox_offset,
                lat_min - bbox_offset,
                lat_max + bbox_offset)

    def __read_data(self, files_path, csv_filenames):
        """
        Create Network node objects with given CSV log file data
        :return:
        """
        for csv_filename in csv_filenames:
            self.network_nodes.append(Node(files_path, csv_filename, self.indoor_mode))

    def __network_draw(self, color_map='white'):
        """
        Clear&Draw graph object
        :param color_map:
        :return:
        """

        colors = []
        path_colors = []

        for (_, _, attrib_dict) in list(self.nx_graph.edges.data()):
            colors.append(attrib_dict['color'])

        for (_, _, attrib_dict) in list(self.nx_data_path.edges.data()):
            path_colors.append(attrib_dict['color'])

        if not self.indoor_mode:
            # draw nodes
            nx.draw(self.nx_graph, nx.get_node_attributes(self.nx_graph, 'pos'), with_labels=True,
                    node_color=color_map,
                    edge_color=colors,
                    edgecolors="r",
                    style="dotted",
                    width=1,
                    node_size=200,
                    font_size=8)

            # draw trace of node movement (selected node)
            nx.draw(self.nx_trace, nx.get_node_attributes(self.nx_trace, 'pos'), with_labels=True,
                    node_color="blue",
                    edgecolors='b',
                    node_size=4,
                    font_size=1)

            # data path visualization
            nx.draw(self.nx_data_path, pos=nx.circular_layout(self.nx_graph), with_labels=True,
                    edge_color=path_colors,
                    edgecolors='b',
                    node_size=4,
                    width=5,
                    font_size=1)

            # draw edge line labels
            nx.draw_networkx_edge_labels(self.nx_graph,
                                         nx.get_node_attributes(self.nx_graph, 'pos'),
                                         edge_labels=self.edge_labels,
                                         font_color="green",
                                         label_pos=0.8,
                                         font_size=7)
        else:
            # draw nodes
            nx.draw(self.nx_graph, pos=nx.circular_layout(self.nx_graph), with_labels=True,
                    node_color=color_map,
                    edge_color=colors,
                    edgecolors="r",
                    style="dotted",
                    width=1,
                    node_size=200,
                    font_size=8)

            # data path visualization
            nx.draw(self.nx_data_path, pos=nx.circular_layout(self.nx_graph), with_labels=True,
                    edge_color=path_colors,
                    edgecolors='b',
                    node_size=4,
                    width=5,
                    font_size=1)

            # draw edge line labels
            nx.draw_networkx_edge_labels(self.nx_graph, pos=nx.circular_layout(self.nx_graph),
                                         edge_labels=self.edge_labels,
                                         font_color="green",
                                         label_pos=0.8,
                                         font_size=7)

        pause(LOOP_SECONDS)

        plt.show()

        if self.pause:
            while self.pause:
                pause(LOOP_SECONDS)

    @staticmethod
    def __on_close(event):
        """
        Handle close event
        :param event:
        :return:
        """
        if event.name == "close_event":
            print('Closed Figure!')
            sys.exit(0)

    def __on_key_press_event(self, event):
        """
        keypress event handler
        :param event:
        :return:
        """
        print('key event ' + event.key)
        if event.key == " ":
            if self.pause:
                self.pause = False
            else:
                self.pause = True

    @staticmethod
    def __mac_value_csv_to_set(csv):
        """
        Parse CSV string
        :param csv:
        :return to_set:
        """
        to_set = {}
        data_list = csv.split(";")

        for valuepair in data_list:
            _mac, _value = valuepair.split(" ")[0].split(",")
            to_set[_mac] = str(_value)

        return to_set

    def collect_active_path(self, _from, _to):
        """
        Collect active path information from neighbours list.
        Recursively called.

        """
        # print(f"call {_from=} -> {_to=}")
        for node in self.network_nodes:
            if node.my_mac == _from:
                # print(f"FOUND {node.my_mac=}")
                self.active_path_list.append(_from)
                origs = [pair.split(",") for pair in node.get_originator().split(";")]
                for pair in origs:
                    # pair syntax = [to, nexthop]
                    # this is for recursive return if search complete
                    if self.active_path_list[-1] == _to:
                        return
                    if pair[0] == _to:
                        # print(f"to = {pair[0]}\nnexthop = {pair[1]}")
                        # return if nexthop is to-target
                        if pair[1] == _to:
                            self.active_path_list.append(pair[1])
                        else:  # continue searching recursively
                            _from = pair[1]
                            self.collect_active_path(_from, _to)

    def __collect_node_data(self):
        """
        Collect node data from log file
        :return:
        """

        # create nodes and edges
        self.__collect_data_and_create_nodes()

        # create moving node path
        if not self.indoor_mode:
            # moved path
            letters = string.ascii_letters
            for node in self.network_nodes:
                if node.my_mac == self.trace_this_mac:
                    #for seconds in np.arange(self.sec_offset_from_start-10, self.sec_offset_from_start, LOOP_SECONDS):
                    #    node.update_row_offset_from_seconds_offset(seconds)
                    for r, __ in enumerate(range(0, self.sec_offset_from_start)):
                        node.update_row_offset_from_seconds_offset(r)
                        self.nx_trace.add_node(''.join(random.choice(letters) for __ in range(10)),
                                               pos=node.get_location_utm())

        # collect data path info and create edges
        if len(self.active_path_macs) > 0:
            self.__collect_data_path_and_create_edges()

        # increase loop count for next round
        self.sec_offset_from_start += LOOP_SECONDS

    def __collect_data_path_and_create_edges(self):
        """
        Collect data path information and create edge lines respectively
        """
        self.active_path_list = []
        self.collect_active_path(self.active_path_macs[0], self.active_path_macs[1])
        from_to = self.active_path_list
        self.active_path_list = []
        self.collect_active_path(self.active_path_macs[1], self.active_path_macs[0])
        to_from = self.active_path_list

        print("\nData path tracking from originator table:")
        print(f"from-to: {from_to}")
        print(f"to-from: {to_from}")

        for index, m in enumerate(from_to):
            if index < len(from_to) - 1:
                self.nx_data_path.add_edge(m, from_to[index + 1], color='green', weight=4)

        for index, m in enumerate(to_from):
            if index < len(to_from) - 1:
                self.nx_data_path.add_edge(m, to_from[index + 1], color='red', weight=4)

    def __collect_data_and_create_nodes(self):
        """
        Collect data ana create node objects with edges
        """
        pylab.clf()
        self.nx_graph.clear()
        self.nx_trace.clear()
        self.nx_data_path.clear()
        self.edge_labels = {}

        # static node positions
        for node in self.network_nodes:
            node.update_row_offset_from_seconds_offset(self.sec_offset_from_start)

            print(f"\n--- Neighbour info from {node.my_mac}: ---")

            if self.indoor_mode:
                self.nx_graph.add_node(node.get_mac(), pos=nx.circular_layout(self.nx_graph))
            else:
                self.nx_graph.add_node(node.get_mac(), pos=node.get_location_utm())
                if self.rxtx:
                    pylab.text(node.get_location_utm()[0], node.get_location_utm()[1] + 10,
                           f"TX:{round(node.get_tx_throughput() / 1000)}\n"
                           f"RX:{round(node.get_rx_throughput() / 1000)} (kbit/s)")

            # check neighbour_list and add/remove connections
            nodes = node.get_neighbours().split(";")
            for _ in nodes:
                _mac, _seen = _.split(",")
                # add active connection
                if float(_seen) < 2.0 and (node.my_mac, _mac) not in self.nx_graph.edges:
                    self.nx_graph.add_edge(node.my_mac, _mac, color='gray', weight=4)

                # if not seen 2s, remove connection
                if float(_seen) >= 2.0 and (node.my_mac, _mac) in self.nx_graph.edges:
                    self.nx_graph.remove_edge(node.my_mac, _mac)

            mac_and_rssi = self.__mac_value_csv_to_set(node.get_rssi())
            mac_and_mcs = self.__mac_value_csv_to_set(node.get_txmcs())

            print("MAC             rssi(noise)mcs_tx")
            for n in self.nx_graph.edges:
                n1, n2, _ = n
                if n1 == node.my_mac and n2 in mac_and_rssi:
                    label = f"{mac_and_rssi[n2]}({node.get_noise()}){mac_and_mcs[n2]}"
                    print(f"{n2} {label}")
                    self.edge_labels[(node.my_mac, n2)] = label

    def __sync_node_timestamps(self):
        """
        Find the laziest bootup time and sync log files with offsets
        :return:
        """

        # find laziest device to bootup time
        laziest = 0

        for node in self.network_nodes:
            laziest = node.get_time_stamp_in_s(0) if node.get_time_stamp_in_s(
                0) > laziest else laziest
            # print(node.my_mac, node.get_time_stamp_in_s(0), laziest)

        print(f"\nFind the laziest log timestamp: {laziest}")
        print("Sync. all nodes....\n")
        # update offset to quickest ones
        for node in self.network_nodes:
            print(f"{node.my_mac}\n  my first timestamp: {node.get_time_stamp_in_s(0)}")
            print(f"  Calculated diff {laziest - node.get_time_stamp_in_s(0)} seconds")
            node.set_time_offset(laziest - node.get_time_stamp_in_s(0))

    def __update_bbox(self):
        """
        Update BBOX from node coordinates
        :return:
        """
        self.__bbox = self.__bbox_of_network()
        lonmin, lonmax, latmin, latmax = self.__bbox
        pylab.xlim(latmax, latmin)
        pylab.ylim(lonmax, lonmin)

    def execute(self, folder_path, filenames, trace_mac):
        """
        Execute plotter with given log file.

        :param folder_path:
        :param filenames:
        :param trace_mac:
        :return:
        """

        self.trace_this_mac = trace_mac

        if filenames != "":
            # read data and create node objects
            self.__read_data(folder_path, filenames)

            pylab.ion()

            # update drawing outline coordinates
            if not self.indoor_mode:
                self.__update_bbox()

            # swap xy to get wgs84 utm correctly oriented
            pylab.gca().invert_xaxis()
            pylab.gca().invert_yaxis()

            # connect event handlers
            fig = pylab.get_current_fig_manager()
            fig.canvas.mpl_connect('close_event', self.__on_close)
            fig.canvas.mpl_connect('key_press_event', self.__on_key_press_event)

            # find sync time
            self.__sync_node_timestamps()
            while True:
                # Collect data from Node objects
                self.__collect_node_data()
                # Draw collected data
                self.__network_draw()
        else:
            print("Please give filename")


def show_usage():
    print('''usage: python ftl_player.py -p <folder> [-m MAC] [-i] [-a MAC,MAC]
    -p path for folder which includes ONLY log files for plotting
    -m travelled path trace for a given MAC address (node)
    -i indoor environment, when GPS use not possible.  Nodes are plotted to the circumference of
       the circle and timestamps are taken from system time instead of GSP time.
    -t rx/tx kbit/s values from each node.  Currently supported in outdoor mode.
    -a active path tracking between given MAC addresses

    Python script will synchronize CSV log files based on the GPS timestamp.  Drawing starts
    from the newest timestamp (old timestamped lines are ignored) with 1s interval.

    example command (python >=3.8):
    python ftl_player.py -p ./test/ -m 22:11:44:55:66:77

    Active keys during plotting:
    spacebar    - pause/resume, zooming possible during pause

    Installation:
    - pip3 install -r requirements.txt

    ''')


if __name__ == '__main__':
    if len(sys.argv) == 1:
        show_usage()
        sys.exit()

    MAC = ""
    PATH = ""
    INDOOR_MODE = False
    RXTX = False
    ACTIVE_PATH = []

    try:
        options, args = getopt.getopt(sys.argv[1:], 'itp:m:a:', [])
        for opt, arg in options:
            if opt in '-m':
                MAC = arg
            if opt in '-p':
                PATH = arg
            if opt in '-i':
                INDOOR_MODE = True
            if opt in '-t':
                RXTX = True
            if opt in '-a':
                ACTIVE_PATH = arg
        if PATH == "":
            raise getopt.error("no path")
        file_list = os.listdir(PATH)
    except getopt.error:
        show_usage()
        sys.exit(2)

    runner = NodeNetwork(INDOOR_MODE, ACTIVE_PATH, RXTX)
    runner.execute(PATH, file_list, MAC)
