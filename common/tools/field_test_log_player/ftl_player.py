#!/usr/bin/python3
"""
Plot static and moving node
"""
import os
import getopt
import random
import string
import sys
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import pandas as pd
import pylab
import utm
from matplotlib.pyplot import pause

LOOP_SECONDS = 1


class Node:
    """
    Single node data collector
    """

    def __init__(self, filename):
        # imported data lists from csv log file
        self.__f_neighbour_list = list()
        self.__f_originator_list = list()
        self.__f_timestamps = list()
        self.__f_gps_timestamps = list()
        self.__f_lat_loc = list()
        self.__f_lon_loc = list()
        self.__f_rssi = list()
        self.__f_noise = list()
        self.__f_txmcs = list()

        self.my_mac = None  # node MAC address
        self.__gps_matched_row_offset = 0  # row offset
        self.__gps_matched_time_s = 0  # offset in seconds to the laziest one
        try:
            self.__load_data_from_file(filename)
        except:
            print(f"Can't load values from: {filename}")
            exit()

    def __load_data_from_file(self, filename):
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

        self.my_mac = filename.split("_")[-1].split(".")[0]
        df = pd.read_csv(filename)

        df.drop(columns=["channel", "txpower [dBm]",
                         "RX MCS [MAC,MCS;MAC,MCS ...]",
                         "TX throughput [Bits/s]", "altitude", "PDOP",
                         "cpu temp [mC]", "wifi temp [mC]", "tmp100 [mC]",
                         "battery voltage [uV]", "battery current [uA]", "nRF voltage [mV]",
                         "nRF current [mA]", "3v3 voltage [mV]", "3v3 current [mA]",
                         "RX throughput [Bits/s]"], inplace=True)

        df.drop(df[df.latitude < -999].index, inplace=True)
        df.drop(df[df.latitude == 0].index, inplace=True)
        df[lat_name].replace('', np.nan, inplace=True)
        df.dropna(subset=[lat_name], inplace=True)
        df[lon_name].replace('', np.nan, inplace=True)
        df.dropna(subset=[lon_name], inplace=True)

        df[neighbours_name].replace('', np.nan, inplace=True)
        df.dropna(subset=[neighbours_name], inplace=True)
        df[originators_name].replace('', np.nan, inplace=True)
        df.dropna(subset=[originators_name], inplace=True)

        df[gps_time].replace('', np.nan, inplace=True)
        df.dropna(subset=[gps_time], inplace=True)

        self.__f_neighbour_list = df[neighbours_name].values.tolist()
        self.__f_originators_list = df[originators_name].values.tolist()
        self.__f_timestamps = df[timestamp_name].values.tolist()
        df[gps_time] = pd.to_datetime(df[gps_time])
        # to seconds
        self.__f_gps_timestamps = [ns / 1000000000 for ns in df[gps_time].values.tolist()]
        self.__f_lat_loc = df[lat_name].values.tolist()
        self.__f_lon_loc = df[lon_name].values.tolist()
        self.__f_rssi = df[rssi_name].values.tolist()
        self.__f_noise = df[noise_name].values.tolist()
        self.__f_txmcs = df[txmcs_name].values.tolist()

    def get_location_utm(self):
        """
        get location from synced time offset point
        :return:
        """
        easting, northing, zone_number, zone_letter = \
            utm.from_latlon(self.__f_lat_loc[self.__gps_matched_row_offset],
                            self.__f_lon_loc[self.__gps_matched_row_offset])
        return easting, northing

    def get_rssi(self):
        """
        get rssi from synced time offset point
        :return:
        """
        return self.__f_rssi[self.__gps_matched_row_offset]

    def get_noise(self):
        """
        get noise from synced time offset point
        :return:
        """
        return self.__f_noise[self.__gps_matched_row_offset]

    def get_txmcs(self):
        """
        get txmcs from synced time offset point
        :return:
        """
        return self.__f_txmcs[self.__gps_matched_row_offset]

    def get_neighbours(self):
        """
        get neighbours from synced time offset point
        :return:
        """
        return self.__f_neighbour_list[self.__gps_matched_row_offset]

    def get_mac(self):
        """
        get mac from synced time offset point
        :return:
        """
        return self.my_mac

    def get_gps_timestamp_in_s(self, row):
        """
        get gps timestamp_s from synced time offset point
        :return:
        """
        return self.__f_gps_timestamps[row]

    def update_row_offset_from_seconds_offset(self, seconds_offset):
        """
        called every update round to check drifting
        :param seconds_offset:
        :return:
        """
        for index, n in enumerate(self.__f_gps_timestamps):
            if n > self.__gps_matched_time_s + seconds_offset:
                self.__gps_matched_row_offset = index
                # print(index, n)
                return

    def set_gps_time_offset(self, seconds):
        """
        Called in startup to calculate offset to log start
        :param seconds:
        :return:
        """
        start = self.get_gps_timestamp_in_s(0)
        matched = start + seconds

        for index, value in enumerate(self.__f_gps_timestamps):
            if self.get_gps_timestamp_in_s(index) >= matched:
                self.__gps_matched_row_offset = index
                self.__gps_matched_time_s = self.get_gps_timestamp_in_s(index)
                # print(self.my_mac, self.__gps_matched_row_offset, self.__gps_matched_time_s)
                return

        raise "not found"


# pylint: disable=too-many-instance-attributes
class NodeNetwork:
    """
    Plot static and moving nodes with matplotlib
    """

    def __init__(self):
        self.nx_edges_list = []
        self.nx_nodes_list = []
        self.nx_graph = nx.MultiDiGraph()
        self.nx_path = nx.MultiDiGraph()
        self.sec_offset_from_start = 0
        self.trace_this_mac = ""
        self.network_nodes = []
        self.__bbox = None  # outlines for drawing
        self.pause = False

    def __bbox_of_network(self):
        """
        Bounding Box with wgs-84 utm coordinates
        :param:
        :return:
        """
        BBOX_OFFSET = 10

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

        return lon_min - BBOX_OFFSET, lon_max + BBOX_OFFSET, \
               lat_min - BBOX_OFFSET, lat_max + BBOX_OFFSET

    def __read_data(self, path, csv_filenames):
        """
        Create Network node objects with given CSV log file data
        :return:
        """
        for csv_filename in csv_filenames:
            self.network_nodes.append(Node(f"{path}/{csv_filename}"))

    def __network_draw(self, color_map='white'):
        """
        Clear&Draw graph object
        :param color_map:
        :return:
        """
        pylab.clf()
        colors = []

        for (u, v, attrib_dict) in list(self.nx_graph.edges.data()):
            colors.append(attrib_dict['color'])

        # draw nodes
        nx.draw(self.nx_graph, nx.get_node_attributes(self.nx_graph, 'pos'), with_labels=True,
                node_color=color_map,
                edge_color=colors,
                edgecolors="r",
                style="dotted",
                width=1,
                node_size=200,
                font_size=8)

        # draw edge line rssi values
        nx.draw_networkx_edge_labels(self.nx_graph, nx.get_node_attributes(self.nx_graph, 'pos'),
                                     edge_labels=self.edge_labels,
                                     font_color="green",
                                     label_pos=0.8,
                                     font_size=7)

        # draw path nodes
        nx.draw(self.nx_path, nx.get_node_attributes(self.nx_path, 'pos'), with_labels=True,
                node_color="blue",
                edgecolors='b',
                node_size=4,
                font_size=1)

        fig = pylab.get_current_fig_manager()
        fig.canvas.mpl_connect('close_event', self.__on_close)
        fig.canvas.mpl_connect('key_press_event', self.__on_key_press_event)

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
            mac, value = valuepair.split(" ")[0].split(",")
            to_set[mac] = str(value)

        return to_set

    def __collect_node_data(self):
        """
        Collect node data from log file
        :return:
        """
        self.nx_graph.clear()
        self.nx_path.clear()
        self.edge_labels = {}

        # static node positions
        for node in self.network_nodes:

            node.update_row_offset_from_seconds_offset(self.sec_offset_from_start)

            print("------------")
            print(node.my_mac + ":")
            print("------------")
            self.nx_graph.add_node(node.get_mac(), pos=node.get_location_utm())

            # check neighbour_list and add/remove connections
            nodes = node.get_neighbours().split(";")
            for _ in nodes:
                mac, seen = _.split(",")
                # add active connection
                if float(seen) < 2.0 and (node.my_mac, mac) not in self.nx_graph.edges:
                    self.nx_graph.add_edge(node.my_mac, mac, color='gray', weight=4)

                # if not seen 2s, remove connection
                if float(seen) >= 2.0 and (node.my_mac, mac) in self.nx_graph.edges:
                    self.nx_graph.remove_edge(node.my_mac, mac)

            mac_and_rssi = self.__mac_value_csv_to_set(node.get_rssi())
            print(mac_and_rssi)
            mac_and_mcs = self.__mac_value_csv_to_set(node.get_txmcs())
            print(mac_and_mcs)

            for n in self.nx_graph.edges:
                n1, n2, n3 = n
                if n1 == node.my_mac and n2 in mac_and_rssi:
                    label = f"{mac_and_rssi[n2]}({node.get_noise()}){mac_and_mcs[n2]}"
                    print(f"{n2} {label}")
                    self.edge_labels[(node.my_mac, n2)] = label

        # moved path
        letters = string.ascii_letters

        for node in self.network_nodes:
            if node.my_mac == self.trace_this_mac:
                for r, __ in enumerate(range(0, self.sec_offset_from_start)):
                    node.update_row_offset_from_seconds_offset(r)
                    self.nx_path.add_node(''.join(random.choice(letters) for __ in range(10)),
                                          pos=node.get_location_utm())

        self.sec_offset_from_start += LOOP_SECONDS

    def __sync_node_gps_times(self):
        """
        Find the laziest bootup time and sync log files with offsets
        :return:
        """

        # find laziest device to bootup time
        laziest = 0

        for node in self.network_nodes:
            laziest = node.get_gps_timestamp_in_s(0) if node.get_gps_timestamp_in_s(0) > laziest else laziest

        # update offset to quickest ones
        for node in self.network_nodes:
            node.set_gps_time_offset(laziest - node.get_gps_timestamp_in_s(0))

    def __update_bbox(self):
        """
        Update BBOX from node coordinates
        :return:
        """
        self.__bbox = self.__bbox_of_network()
        lonmin, lonmax, latmin, latmax = self.__bbox
        pylab.xlim(latmax, latmin)
        pylab.ylim(lonmax, lonmin)

    def execute(self, path, filenames, mac):
        """
        Execute plotter with given log file.

        :param path:
        :param filenames:
        :return:
        """

        self.trace_this_mac = mac

        if filenames != "":
            # read data and create node objects
            self.__read_data(path, filenames)

            pylab.ion()

            # update drawing outline coordinates
            self.__update_bbox()

            # swap xy to get wgs84 utm correctly oriented
            pylab.gca().invert_xaxis()
            pylab.gca().invert_yaxis()

            # find sync time
            self.__sync_node_gps_times()
            while True:
                # Collect data from Node objects
                self.__collect_node_data()
                # Draw collected data
                self.__network_draw()
        else:
            print("Please give filename")


def show_usage():
    print('''usage: python ftl_player.py [-f] [-m]
    -p path for folder which includes ONLY log files for plotting
    -m travelled path trace for a given MAC address (node)

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
        exit()

    mac = ""
    path = ""

    try:
        options, args = getopt.getopt(sys.argv[1:], 'p:m:', [])
        for o, a in options:
            if o == '-m':
                mac = a
            if o == '-p':
                path = a
        if path == "":
            raise "error"
        file_list = os.listdir(path)
    except:
        show_usage()
        sys.exit(2)

    runner = NodeNetwork()
    runner.execute(path, file_list, mac)
