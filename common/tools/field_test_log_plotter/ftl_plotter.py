import sys
import getopt
import os  # For directory / file handling purposes
import logging

import pandas as pd
import matplotlib.pyplot as plt
import geopy.distance as geodist
from tabulate import tabulate
import numpy as np
from ipyleaflet import Map, Marker, AntPath, AwesomeIcon, MeasureControl
from ipywidgets import Layout

# Local imports
from constants import *

# Create logger
logger = logging.getLogger(__name__)
# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
# add ch to logger
logger.addHandler(ch)


def show_plots():
    plt.show()


class FieldTestLogPlotter:
    """
    Field test log plotter class.
    """
    def __init__(self, filename, throughput_unit='b'):
        """
        Creates a pandas dataframe from given input file.

        :param filename: Filename of field test log file (csv).
        :param throughput_unit: b, Kb or Mb.
       """
        self.filename = filename
        self.df = None  # To be set later
        self.mac_list = []
        self.coordinates = []
        self.is_base = False
        self.__base_latitude = 0
        self.__base_longitude = 0
        self.__homedir = os.path.expanduser('~')
        self.throughput_units = throughput_unit
        self.__create_dataframe()

    def __create_dataframe(self):
        """

        :return: None
        """
        # Create dataframe from csv file.
        self.df = pd.read_csv(self.filename)
        # Debug
        logger.debug(tabulate(self.df, headers='keys', tablefmt='psql'))

        # Convert invalid GPS coordinates first to NaN. Assumption:
        # invalid values exists only at the beginning of the log.
        self.df['latitude'].replace(-999999, np.NaN, inplace=True)
        self.df['longitude'].replace(-999999, np.NaN, inplace=True)
        self.df['latitude'].replace(0, np.NaN, inplace=True)
        self.df['longitude'].replace(0, np.NaN, inplace=True)

        # Then replace NaN values with first valid coordinates.
        row_index = self.df['latitude'].first_valid_index()
        if row_index:
            self.__base_latitude = self.df.loc[row_index, 'latitude']
            self.__base_longitude = self.df.loc[row_index, 'longitude']
        self.df['latitude'].replace(np.NaN, self.__base_latitude, inplace=True)
        self.df['longitude'].replace(np.NaN, self.__base_longitude, inplace=True)
        # Initialize invalid PDOP
        self.df['PDOP'].replace(0, 99.99, inplace=True)
        row_index = self.df['GPS time'].first_valid_index()
        if row_index:
            gpstime = self.df.loc[row_index, 'GPS time']
        else:
            gpstime = 0
        self.df['GPS time'].replace(np.NaN, gpstime, inplace=True)

        # Replace non-existing RSSI and/or MCS values with some defaults
        self.df['rssi [MAC,dBm;MAC,dBm ...]'].replace(np.nan,
                                                      '00:00:00:00:00:00,-115 [-119, -118, -117]',
                                                      inplace=True)
        self.df['RX MCS [MAC,MCS;MAC,MCS ...]'].replace(np.nan,
                                                        '00:00:00:00:00:00,-1',
                                                        inplace=True)
        self.df['TX MCS [MAC,MCS;MAC,MCS ...]'].replace(np.nan,
                                                        '00:00:00:00:00:00,-1',
                                                        inplace=True)
        # Drop any DF lines where timestamp is NaN
        self.df.dropna(subset=['Timestamp'], inplace=True)
        # Reindex DF after any possible line drops
        self.df.reset_index(drop=True, inplace=True)

        if self.df.empty:
            raise Exception('Dataframe is empty!')

        # Debug
        logger.debug(tabulate(self.df, headers='keys', tablefmt='psql'))

        # Calculate distance to base station and total distance moved around.
        self.__calculate_distances((self.__base_latitude, self.__base_longitude))
        self.__is_base_device()

        # Convert date time formatted information to relative time stamps in seconds.
        self.df['Timestamp'] = pd.to_datetime(self.df['Timestamp'])
        self.df['Timestamp'] = pd.to_timedelta(self.df['Timestamp'] - self.df['Timestamp'][0])
        self.df['Timestamp'] = self.df['Timestamp'].dt.total_seconds()

        self.__parse_rssi_data()
        self.__parse_mcs_class_data('RX MCS')
        self.__parse_mcs_class_data('TX MCS')

        # Add calculated SNR (RSSI - Noise) to dataframe
        # self.df['snr [dB]'] = self.df['rssi [dBm]'] - self.df['noise [dBm]']

        # Convert units to more readable format
        self.__convert_units()
        # Debug
        logger.debug('datatypes: ', self.df.dtypes)
        logger.info(tabulate(self.df, headers='keys', tablefmt='psql'))

        # Write generated dataframe to CSV file
        # self.df.to_csv('generated_dataframe.csv')
        df2 = self.df[['latitude', 'longitude']].copy()
        self.coordinates = df2.values.tolist()
        # Debug coordinates
        logger.debug(self.coordinates)

    def __parse_rssi_data(self):
        """
        Parse data from 'rssi [MAC,dBm;MAC,dBm ...]' into separate
        device (mac) specific columns.

        Example data in rssi [MAC,dBm;MAC,dBm ...]:
        04:f0:21:a8:6a:b8,-54 [-61, -56, -61];00:30:1a:4f:17:65,-41 [-44, -46, -47]
        :return: None
        """
        # Remove square brackets and extra spaces to create data
        # column where delimiters are commas and semicolon.
        self.df['rssi pattern'] = self.df['rssi [MAC,dBm;MAC,dBm ...]'].str.replace(r' \[|, ', ',',
                                                                                    regex=True)
        self.df['rssi pattern'] = self.df['rssi pattern'].str.replace(r'\]', '', regex=True)
        # Iterate through dataframe rows to parse newly created
        # 'rssi pattern' column to split data into sub columns.
        for df_row_index in self.df.index:
            rssi_dataset = self.df['rssi pattern'].str.split(';')[df_row_index]
            for index, rssi_set in enumerate(rssi_dataset):
                rssi_data = rssi_set.split(',')
                _mac = rssi_data[0]
                if _mac not in self.mac_list:
                    self.mac_list.append(_mac)
                # Device (mac) specific column names:
                rssi_dbm = _mac + ' rssi [dBm]'
                rssi_ant0_dbm = _mac + ' rssi ant0 [dBm]'
                rssi_ant1_dbm = _mac + ' rssi ant1 [dBm]'
                rssi_ant2_dbm = _mac + ' rssi ant2 [dBm]'

                # Create empty columns to enable later assignments using index
                if rssi_dbm not in self.df.columns:
                    self.df[rssi_dbm] = None
                if rssi_ant0_dbm not in self.df.columns:
                    self.df[rssi_ant0_dbm] = None
                if rssi_ant1_dbm not in self.df.columns:
                    self.df[rssi_ant1_dbm] = None
                if rssi_ant2_dbm not in self.df.columns:
                    self.df[rssi_ant2_dbm] = None
                # Assign data using row index
                self.df.loc[df_row_index, rssi_dbm] = pd.to_numeric(rssi_data[1])
                self.df.loc[df_row_index, rssi_ant0_dbm] = pd.to_numeric(rssi_data[2])
                self.df.loc[df_row_index, rssi_ant1_dbm] = pd.to_numeric(rssi_data[3])
                self.df.loc[df_row_index, rssi_ant2_dbm] = pd.to_numeric(rssi_data[4])

        # Convert string/object type data to float
        for _mac in self.mac_list:
            rssi_dbm = _mac + ' rssi [dBm]'
            rssi_ant0_dbm = _mac + ' rssi ant0 [dBm]'
            rssi_ant1_dbm = _mac + ' rssi ant1 [dBm]'
            rssi_ant2_dbm = _mac + ' rssi ant2 [dBm]'
            # Convert data to float
            self.df = self.df.astype({rssi_dbm: float,
                                      rssi_ant0_dbm: float,
                                      rssi_ant1_dbm: float,
                                      rssi_ant2_dbm: float}, errors='raise')
            # Set NaN values to OUT_OF_SCALE_RSSI that is clearly out of normal scale.
            self.df[rssi_dbm].replace(np.nan, OUT_OF_SCALE_RSSI, inplace=True)

    def __parse_mcs_class_data(self, rx_or_tx):
        """
        Parse RX / TX MCS data into separate columns.
        :param rx_or_tx: 'RX MCS' or 'TX MCS'
        :return: None
        """
        csv_column_name = rx_or_tx + ' [MAC,MCS;MAC,MCS ...]'
        pre_parsed_df_column = rx_or_tx + ' pattern'

        # Remove square brackets and extra spaces to create data column where
        # delimiters are commas and semicolon.
        self.df[pre_parsed_df_column] = self.df[csv_column_name].str.replace(r' \[|, ', ',',
                                                                             regex=True)
        self.df[pre_parsed_df_column] = self.df[pre_parsed_df_column].str.replace(r'\]', '',
                                                                                  regex=True)
        # Iterate through dataframe rows to parse RX [MAC,MCS;MAC,MCS ...]
        for df_row_index in self.df.index:
            mcs_dataset = self.df[pre_parsed_df_column].str.split(';')[df_row_index]
            for index, mcs_set in enumerate(mcs_dataset):
                mcs_data = mcs_set.split(',')
                _mac = mcs_data[0]
                if _mac not in self.mac_list:
                    self.mac_list.append(_mac)
                # Device (mac) specific column names:
                mcs_class = _mac + ' ' + rx_or_tx
                # Create empty columns to enable later assignments using index
                if mcs_class not in self.df.columns:
                    self.df[mcs_class] = None
                    self.df[mcs_class] = pd.to_numeric(self.df[mcs_class])
                # Assign data using row index
                # self.df[mcs_class][df_row_index] = mcs_data[1]
                self.df.loc[df_row_index, mcs_class] = mcs_data[1]
        # Convert string/object type data to float
        for _mac in self.mac_list:
            mcs_class = _mac + ' ' + rx_or_tx
            # Convert data to float
            self.df = self.df.astype({mcs_class: float}, errors='raise')
            # Replace NaN values with -1 that is out of normal scale.
            self.df[mcs_class].replace(np.nan, -1, inplace=True)

    def __is_base_device(self):
        """
        Determines if logged device has been so-called base device.
        Decision is made based on filename, max distance and median top
        speed.

        :return: None
        """
        # Detect movement from highest 15 speed values
        top_speeds = self.df['speed [km/h]'].nlargest(n=15)
        median_speed = top_speeds.median()
        max_distance = self.df['distance [m]'].max()

        if self.filename.find('base') != -1:
            self.is_base = True
        elif max_distance <= 10:
            self.is_base = True
        elif median_speed < 3:
            self.is_base = True
        else:
            self.is_base = False
        # Debug info
        logger.info('is_base: %s, max_distance: %s, top speed median:  %s',
                    self.is_base, max_distance, median_speed)

    def __calculate_distances(self, base_lat_lon):
        """
        Calculates distance from the DUT to base station as well as total
        distance traveled based on GPS coordinates.

        :param base_lat_lon: (latitude, longitude) of the base station
        :return: None
        """
        # Calculate distance to base station.
        self.df['distance [m]'] = self.df.apply(lambda row:
                                                geodist.geodesic(base_lat_lon,
                                                                 (row['latitude'],
                                                                  row['longitude'])).meters,
                                                axis=1)

        # Calculate GPS time delta for speed estimations
        self.df['GPS time delta'] = pd.to_datetime(self.df['GPS time'])
        self.df['GPS time delta'] = pd.to_timedelta(self.df['GPS time delta'] -
                                                    self.df['GPS time delta'][0])
        self.df['GPS time delta'] = self.df['GPS time delta'].dt.total_seconds()

        # Calculate total distance (i.e. back and forth route) traveled.
        distance_traveled = []
        speed = []

        for index in self.df.index:
            # First we calculate distance to base station.
            if index == 0:
                distance_traveled.append(geodist.geodesic(base_lat_lon,
                                                          (self.df['latitude'][index],
                                                           self.df['longitude'][index])).meters)
                speed.append(np.NaN)
            # Next we add sum of distances between consequent rows.
            else:
                distance = geodist.geodesic((self.df['latitude'][index - 1],
                                             self.df['longitude'][index - 1]),
                                            (self.df['latitude'][index],
                                             self.df['longitude'][index])).meters
                distance_traveled.append(distance_traveled[index - 1] + distance)
                # Determine speed
                timedelta = self.df['GPS time delta'][index]-self.df['GPS time delta'][index-1]
                if distance_traveled[index-1] == 0:
                    speed.append(0)  # Eliminate too high speed for the first measured distance
                else:
                    speed.append(distance / timedelta * 3.6)
        # Add new columns into dataframe
        self.df['distance traveled [m]'] = distance_traveled
        self.df['speed [km/h]'] = speed

    def __convert_units(self):
        """
        Converts dataset units and renames dataframe columns accordingly.
        For example: mC => C, µV and uA to V / A and so on.
        :return: None
        """
        # Convert millidegree Celsius values to Celsius
        self.df['cpu temp [mC]'] = self.df['cpu temp [mC]'] / 1000
        self.df.rename(columns={'cpu temp [mC]': CPU_TEMP}, inplace=True)
        # In some logs Wi-Fi card temperature has been NaN. Treat it as zero.
        self.df.rename(columns={'wifi temp [mC]': WIFI_TEMP}, inplace=True)
        self.df[WIFI_TEMP].fillna(0, inplace=True)
        self.df[WIFI_TEMP] = self.df[WIFI_TEMP] / 1000

        self.df['tmp100 [mC]'] = self.df['tmp100 [mC]'] / 1000
        self.df.rename(columns={'tmp100 [mC]': TMP100}, inplace=True)

        # Convert µV and uA to V / A
        self.df['battery voltage [uV]'] = self.df['battery voltage [uV]'] / 1000000
        self.df.rename(columns={'battery voltage [uV]': BATT_VOLTAGE}, inplace=True)
        self.df['battery current [uA]'] = abs(self.df['battery current [uA]'] / 1000000)
        self.df.rename(columns={'battery current [uA]': BATT_CURRENT}, inplace=True)

        # Convert mV and mA to V / A
        self.df['nRF voltage [mV]'] = self.df['nRF voltage [mV]'] / 1000
        self.df.rename(columns={'nRF voltage [mV]': NRF_VOLTAGE}, inplace=True)
        self.df['nRF current [mA]'] = abs(self.df['nRF current [mA]'] / 1000)
        self.df.rename(columns={'nRF current [mA]': NRF_CURRENT}, inplace=True)
        self.df['3v3 voltage [mV]'] = self.df['3v3 voltage [mV]'] / 1000
        self.df.rename(columns={'3v3 voltage [mV]': MPCIE_VOLTAGE}, inplace=True)
        self.df['3v3 current [mA]'] = abs(self.df['3v3 current [mA]'] / 1000)
        self.df.rename(columns={'3v3 current [mA]': MPCIE_CURRENT}, inplace=True)

        tp_divider = 1
        # Convert throughput values
        if self.throughput_units == 'Kb':
            tp_divider = 1000
        elif self.throughput_units == 'Mb':
            tp_divider = 1000000
        if tp_divider > 1:
            # Converting only actual values but not changing column name.
            self.df['RX throughput [Bits/s]'] = self.df['RX throughput [Bits/s]'] / tp_divider
            self.df['TX throughput [Bits/s]'] = self.df['TX throughput [Bits/s]'] / tp_divider

    def plot_temp_voltage_and_current(self):
        """
        Plots temperatures, voltages and current as a function of time.
        The first y-axis is for CPU, Wi-Fi card and TMP100 sensor temperatures.
        The second y-axis is for battery, RF amd voltages and the third one for current.

        :return: None
        """
        fig, ax_temp = plt.subplots()
        ax_temp.set_facecolor(COLOR_FACECOLOR)
        ax_voltage = ax_temp.twinx()
        ax_current = ax_temp.twinx()

        rspine = ax_current.spines['right']
        rspine.set_position(('axes', 1.17))
        fig.subplots_adjust(right=0.75)

        # Determine x-axis scale
        min_time = min(self.df['Timestamp'])
        max_time = max(self.df['Timestamp'])
        interval = int((max_time - min_time) / 40)
        if interval > 10:
            interval = round(float(interval) / 10) * 10
        elif interval == 0:
            interval = 1
        max_time += interval

        subtitle = '_temperature_voltage_and_current'
        figure_title = self.filename.replace(self.__homedir, '') + subtitle

        # Plot temperatures
        self.df.plot(title=figure_title,
                     ax=ax_temp, x='Timestamp', y=CPU_TEMP, color=COLOR_CPU_TEMP, lw=2)
        self.df.plot(ax=ax_temp, x='Timestamp', y=WIFI_TEMP, color=COLOR_WIFI_TEMP, lw=2)
        self.df.plot(ax=ax_temp, x='Timestamp', y=TMP100, color=COLOR_TMP100, lw=2, grid=True,
                     xlabel=LABEL_TIME_SECONDS, ylabel=LABEL_TEMPERATURE,
                     xticks=(np.arange(min_time, max_time, interval)),
                     yticks=(np.arange(0, 101, 10)),
                     figsize=FIGSIZE)
        # Get dataframe row index according to max distance
        rows = self.df.index[self.df['distance [m]'] == max(self.df['distance [m]'])].tolist()

        # Set vertical split line for turning point. Note! Using first index match.
        ax_temp.axvline((self.df['Timestamp'][rows[0]]), color=COLOR_TURNING_POINT, lw=3,
                        linestyle='--', label='Turning point')
        # Show legend
        ax_temp.legend(bbox_to_anchor=(-0.03, 1.0), loc='upper right')
        # Rotate x-axis tick labels to make them fit better in the picture
        plt.setp(ax_temp.get_xticklabels(), rotation=30, ha='right')

        # Determine 2nd y-axis scale
        voltage_max = max(self.df[BATT_VOLTAGE])
        if voltage_max < max(self.df[NRF_VOLTAGE]):
            voltage_max = max(self.df[NRF_VOLTAGE])
        if voltage_max < max(self.df[MPCIE_VOLTAGE]):
            voltage_max = max(self.df[MPCIE_VOLTAGE])
        voltage_max += 0.25  # 0.25 V margin on top
        interval = 0.5  # 0.5 V interval

        # Plot voltage figures
        self.df.plot(ax=ax_voltage, x='Timestamp', y=BATT_VOLTAGE, color=COLOR_BATT_VOLTAGE)
        self.df.plot(ax=ax_voltage, x='Timestamp', y=NRF_VOLTAGE, color=COLOR_RF_VOLTAGE)
        self.df.plot(ax=ax_voltage, x='Timestamp', y=MPCIE_VOLTAGE, color=COLOR_3V3_VOLTAGE,
                     grid=True, ylabel=LABEL_VOLTAGE, yticks=(np.arange(0, voltage_max, interval)))

        # Label box upper left corner coordinate relative to base plot coordinate
        ax_voltage.legend(bbox_to_anchor=(1.03, 1.0), loc='upper left')

        # Determine 3rd y-axis scale
        curr_max = max(self.df[BATT_CURRENT])
        if curr_max < max(self.df[NRF_CURRENT]):
            curr_max = max(self.df[NRF_CURRENT])
        if curr_max < max(self.df[MPCIE_CURRENT]):
            curr_max = max(self.df[MPCIE_CURRENT])
        curr_max += 0.25  # 0.25 A margin on top
        interval = 0.5  # 0.5 A interval

        # Plot current figures
        self.df.plot(ax=ax_current, x='Timestamp', y=BATT_CURRENT, color=COLOR_BATT_CURRENT)
        self.df.plot(ax=ax_current, x='Timestamp', y=NRF_CURRENT, color=COLOR_RF_CURRENT)
        self.df.plot(ax=ax_current, x='Timestamp', y=MPCIE_CURRENT, color=COLOR_3V3_CURRENT,
                     grid=True, ylabel=LABEL_CURRENT, yticks=(np.arange(0, curr_max, interval)))
        # Label box lower left corner coordinate relative to base plot coordinate
        ax_current.legend(bbox_to_anchor=(1.18, 1.0), loc='upper left')
        # Save the figure
        plt.savefig(self.filename + subtitle + ".png", dpi=MY_DPI)

    def plot_temperature_and_current(self):
        """
        Plots temperature and current values as a function of time.
        The first y-axis is for CPU, Wi-Fi card and TMP100 sensor temperatures.
        The second y-axis is for current values.

        :return: None
        """
        fig, ax_temp = plt.subplots()
        ax_temp.set_facecolor(COLOR_FACECOLOR)
        ax_current = ax_temp.twinx()

        # Determine x-axis scale
        min_time = min(self.df['Timestamp'])
        max_time = max(self.df['Timestamp'])
        interval = int((max_time - min_time) / 40)
        if interval > 10:
            interval = round(float(interval) / 10) * 10
        elif interval == 0:
            interval = 1
        max_time += interval

        subtitle = '_temperature_and_current'
        figure_title = self.filename.replace(self.__homedir, '') + subtitle

        # Plot temperatures
        self.df.plot(title=figure_title,
                     ax=ax_temp, x='Timestamp', y=CPU_TEMP, color=COLOR_CPU_TEMP, lw=2)
        self.df.plot(ax=ax_temp, x='Timestamp', y=WIFI_TEMP, color=COLOR_WIFI_TEMP, lw=2)
        self.df.plot(ax=ax_temp, x='Timestamp', y=TMP100, color=COLOR_TMP100, lw=2, grid=True,
                     xlabel=LABEL_TIME_SECONDS, ylabel=LABEL_TEMPERATURE,
                     xticks=(np.arange(min_time, max_time, interval)),
                     yticks=(np.arange(0, 101, 10)),
                     figsize=FIGSIZE)
        # Get dataframe row index according to max distance
        rows = self.df.index[self.df['distance [m]'] == max(self.df['distance [m]'])].tolist()

        # Set vertical split line for turning point. Note! Using first index match.
        ax_temp.axvline((self.df['Timestamp'][rows[0]]), color=COLOR_TURNING_POINT, lw=3,
                        linestyle='--', label='Turning point')
        # Show legend
        ax_temp.legend(bbox_to_anchor=(-0.03, 1.0), loc='upper right')
        # Rotate x-axis tick labels to make them fit better in the picture
        plt.setp(ax_temp.get_xticklabels(), rotation=30, ha='right')

        # Determine y-axis scale
        curr_max = max(self.df[BATT_CURRENT])
        if curr_max < max(self.df[NRF_CURRENT]):
            curr_max = max(self.df[NRF_CURRENT])
        if curr_max < max(self.df[MPCIE_CURRENT]):
            curr_max = max(self.df[MPCIE_CURRENT])
        curr_max += 0.25  # 0.25 A margin on top
        interval = 0.5  # 0.5 A interval

        # Plot current figures
        self.df.plot(ax=ax_current, x='Timestamp', y=BATT_CURRENT, color=COLOR_BATT_CURRENT)
        self.df.plot(ax=ax_current, x='Timestamp', y=NRF_CURRENT, color=COLOR_RF_CURRENT)
        self.df.plot(ax=ax_current, x='Timestamp', y=MPCIE_CURRENT, color=COLOR_3V3_CURRENT,
                     grid=True, ylabel=LABEL_CURRENT, yticks=(np.arange(0, curr_max, interval)))
        # 2nd y-axis legend
        ax_current.legend(bbox_to_anchor=(1.03, 1.0), loc='upper left')
        # Save the figure
        plt.savefig(self.filename + subtitle + ".png", dpi=MY_DPI)

    def plot_voltage_and_current(self):
        """
        Plots voltage and current values as a function of time.
        The first y-axis is for voltage values and the second y-axis is for current values.

        :return: None
        """
        fig, ax_voltage = plt.subplots()
        ax_voltage.set_facecolor(COLOR_FACECOLOR)
        ax_current = ax_voltage.twinx()

        # Determine x-axis scale
        min_time = min(self.df['Timestamp'])
        max_time = max(self.df['Timestamp'])
        interval = int((max_time - min_time) / 40)
        if interval > 10:
            interval = round(interval / 10) * 10
        elif interval == 0:
            interval = 1
        max_time += interval

        # Determine y-axis scale
        voltage_max = max(self.df[BATT_VOLTAGE])
        if voltage_max < max(self.df[NRF_VOLTAGE]):
            voltage_max = max(self.df[NRF_VOLTAGE])
        if voltage_max < max(self.df[MPCIE_VOLTAGE]):
            voltage_max = max(self.df[MPCIE_VOLTAGE])
        voltage_max += 0.25  # 0.25 V margin on top
        voltage_interval = 0.5  # 0.5 V interval

        subtitle = '_voltage_and_current'
        figure_title = self.filename.replace(self.__homedir, '') + subtitle

        # Plot voltage figures
        self.df.plot(ax=ax_voltage, x='Timestamp', y=BATT_VOLTAGE, color=COLOR_BATT_VOLTAGE)
        self.df.plot(ax=ax_voltage, x='Timestamp', y=NRF_VOLTAGE, color=COLOR_RF_VOLTAGE)
        self.df.plot(ax=ax_voltage, x='Timestamp', y=MPCIE_VOLTAGE, color=COLOR_3V3_VOLTAGE,
                     grid=True, xlabel=LABEL_TIME_SECONDS, ylabel=LABEL_TEMPERATURE,
                     xticks=(np.arange(min_time, max_time, interval)),
                     yticks=(np.arange(0, voltage_max, voltage_interval)),
                     title=figure_title, figsize=FIGSIZE)
        # Get dataframe row index according to max distance
        rows = self.df.index[self.df['distance [m]'] == max(self.df['distance [m]'])].tolist()

        # Set vertical split line for turning point. Note! Using first index match.
        ax_voltage.axvline((self.df['Timestamp'][rows[0]]), color=COLOR_TURNING_POINT, lw=3,
                           linestyle='--', label='Turning point')
        # Label box upper right corner coordinate relative to base plot coordinate
        ax_voltage.legend(bbox_to_anchor=(-0.03, 1.0), loc='upper right')
        # Rotate x-axis tick labels to make them fit better in the picture
        plt.setp(ax_voltage.get_xticklabels(), rotation=30, ha='right')

        # Determine y-axis scale
        curr_max = max(self.df[BATT_CURRENT])
        if curr_max < max(self.df[NRF_CURRENT]):
            curr_max = max(self.df[NRF_CURRENT])
        if curr_max < max(self.df[MPCIE_CURRENT]):
            curr_max = max(self.df[MPCIE_CURRENT])
        curr_max += 0.25  # 0.25 A margin on top
        interval = 0.5  # 0.5 A interval

        # Plot current figures
        self.df.plot(ax=ax_current, x='Timestamp', y=BATT_CURRENT, color=COLOR_BATT_CURRENT)
        self.df.plot(ax=ax_current, x='Timestamp', y=NRF_CURRENT, color=COLOR_RF_CURRENT)
        self.df.plot(ax=ax_current, x='Timestamp', y=MPCIE_CURRENT, color=COLOR_3V3_CURRENT,
                     grid=True, ylabel=LABEL_CURRENT, yticks=(np.arange(0, curr_max, interval)))
        # 2nd y-axis legend
        ax_current.legend(bbox_to_anchor=(1.03, 1.0), loc='upper left')
        # Save the figure
        plt.savefig(self.filename + subtitle + ".png", dpi=MY_DPI)

    def rssi_noise_throughput_and_mcs(self, _mac, x_axis='Timestamp', x_start=None, x_stop=None):
        """
        Plots RSSI sum signal level from heard device, noise level, RX/TX throughput and MCS class
        info as a function of time/distance or total distance. Signal level scale is described
        of first y-axis, throughput values on second y-axis and MCS class values on third y-axis.
        RX and TX throughput values.

        :param _mac: MAC address of the device heard.
        :param x_axis: Name of the dataframe column to be used as X axis value.
                       Default value is 'Timestamp'. Other typical options:
                       'distance traveled [m]' or 'distance [m]'
        :param x_start: Optional x axis slice start value. Default value is None.
        :param x_stop: Optional x axis slice end value. Default value is None.

        :return: None
        """
        if self.is_base and x_axis != 'Timestamp':
            return
        fig, ax_signal_strength = plt.subplots()
        ax_signal_strength.set_facecolor(COLOR_FACECOLOR)
        ax_mcs = ax_signal_strength.twinx()
        ax_throughput = ax_signal_strength.twinx()

        rspine = ax_mcs.spines['right']
        rspine.set_position(('axes', 1.07))
        fig.subplots_adjust(right=0.85)

        if x_start:
            start_idx = self.df[self.df[x_axis].le(float(x_start))].index.max()
        else:
            start_idx = None
        if x_stop:
            stop_idx = self.df[self.df[x_axis].le(float(x_stop))].index.max()
        else:
            stop_idx = None

        # Determine x-axis scale
        min_x_axis_val = min(self.df.iloc[start_idx:stop_idx][x_axis])
        max_x_axis_val = max(self.df.iloc[start_idx:stop_idx][x_axis])
        x_interval = int((max_x_axis_val - min_x_axis_val) / 40)

        if x_interval > 10:
            x_interval = round(x_interval / 10) * 10
        elif x_interval == 0:
            x_interval = 1
        max_x_axis_val += x_interval

        logger.debug('min_x_axis_val %s, max_x_axis_val %s, x_interval %s',
                     min_x_axis_val, max_x_axis_val, x_interval)

        # Device specific column names:
        rssi_dbm = _mac + ' rssi [dBm]'
        rx_mcs = _mac + ' RX MCS'
        tx_mcs = _mac + ' TX MCS'

        if x_axis == 'Timestamp':
            subtitle = '_signal_strength_and_throughput_per_time'
        elif x_axis == 'distance traveled [m]':
            subtitle = '_signal_strength_and_throughput_per_distance_traveled'
        elif x_axis == 'distance [m]':
            subtitle = '_signal_strength_and_throughput_per_distance'
        else:
            subtitle = '_signal_strength_and_throughput_per_distance' + x_axis

        figure_title = self.filename.replace(self.__homedir, '') + subtitle

        # Plot signal strength figures
        self.df.iloc[start_idx:stop_idx].plot(ax=ax_signal_strength, x=x_axis,
                                              y=rssi_dbm, color=COLOR_RSSI, lw=1.5)
        self.df.iloc[start_idx:stop_idx].plot(ax=ax_signal_strength, x=x_axis,
                                              y='noise [dBm]', color=COLOR_NOISE,
                                              grid=True, ylabel=LABEL_SIGNAL_STRENGTH,
                                              xticks=(np.arange(min_x_axis_val,
                                                                max_x_axis_val,
                                                                x_interval)),
                                              yticks=(np.arange(-110, 0, 10)),
                                              title=figure_title, figsize=FIGSIZE)

        if not self.is_base:
            # Get dataframe row index according to max distance
            rows = self.df.index[self.df['distance [m]'] == max(self.df['distance [m]'])].tolist()
            index = rows[0]
            plot_turning_point = False

            if start_idx is None and stop_idx is None:
                plot_turning_point = True
            elif start_idx is None and stop_idx is not None:
                if index < stop_idx:
                    plot_turning_point = True
            elif start_idx is not None and stop_idx is None:
                if index > start_idx:
                    plot_turning_point = True
            else:
                if start_idx < index < stop_idx:
                    plot_turning_point = True
            if plot_turning_point:
                if x_axis == 'Timestamp':
                    # Set vertical split line for turning point.
                    ax_signal_strength.axvline((self.df['Timestamp'][index]),
                                               color=COLOR_TURNING_POINT,
                                               lw=3, linestyle='--', label='Turning point')
                else:
                    ax_signal_strength.axvline((self.df['distance [m]'][index]),
                                               color=COLOR_TURNING_POINT,
                                               linestyle='--', label='Turning point', lw=3)

        # Label box location
        ax_signal_strength.legend(bbox_to_anchor=(0, 1.0), loc='upper left')

        # Rotate x-axis tick labels to make them fit better in the picture
        plt.setp(ax_signal_strength.get_xticklabels(), rotation=30, ha='right')

        # TODO: Consider using fixed throughput scale. Scale (Mbps, Kbps etc.) could be
        #  function parameter. It should ease up comparison of plots from different devices.
        # Determine y-axis scale
        if min(self.df['TX throughput [Bits/s]']) < min(self.df['RX throughput [Bits/s]']):
            min_bitrate = int(min(self.df['TX throughput [Bits/s]']))
        else:
            min_bitrate = int(min(self.df['RX throughput [Bits/s]']))
        if max(self.df['TX throughput [Bits/s]']) > max(self.df['RX throughput [Bits/s]']):
            max_bitrate = int(max(self.df['TX throughput [Bits/s]']))
        else:
            max_bitrate = int(max(self.df['RX throughput [Bits/s]']))
        y_interval = (max_bitrate - min_bitrate) / 10
        max_bitrate += y_interval

        if y_interval == 0:
            return
        elif y_interval > 1:
            y_interval = int(y_interval)

        # Set throughput label names
        rx_throughput_label = 'RX throughput [bit/s]'
        tx_throughput_label = 'TX throughput [bit/s]'
        label_throughput = LABEL_THROUGHPUT
        if self.throughput_units == 'Kb':
            rx_throughput_label = 'RX throughput [Kb/s]'
            tx_throughput_label = 'TX throughput [Kb/s]'
            label_throughput = 'Throughput [Kb/s]'
        elif self.throughput_units == 'Mb':
            rx_throughput_label = 'RX throughput [Mb/s]'
            tx_throughput_label = 'TX throughput [Mb/s]'
            label_throughput = 'Throughput [Mb/s]'

        self.df.iloc[start_idx:stop_idx].plot(ax=ax_throughput, x=x_axis,
                                              y='RX throughput [Bits/s]',
                                              xticks=(np.arange(min_x_axis_val,
                                                                max_x_axis_val, x_interval)),
                                              yticks=(np.arange(min_bitrate,
                                                                max_bitrate, y_interval)),
                                              color=COLOR_RX_THROUGHPUT, lw=2,
                                              label=rx_throughput_label)
        self.df.iloc[start_idx:stop_idx].plot(ax=ax_throughput, title=figure_title,
                                              grid=True, x=x_axis, y='TX throughput [Bits/s]',
                                              xticks=(np.arange(min_x_axis_val,
                                                                max_x_axis_val, x_interval)),
                                              yticks=(np.arange(min_bitrate,
                                                                max_bitrate, y_interval)),
                                              label=tx_throughput_label, figsize=FIGSIZE,
                                              color=COLOR_TX_THROUGHPUT, lw=2,
                                              ylabel=label_throughput)
        # Label box upper right corner coordinate relative to base plot coordinate
        ax_throughput.legend(bbox_to_anchor=(1.0, 1.0), loc='upper right')

        self.df.iloc[start_idx:stop_idx].plot(kind='scatter', ax=ax_mcs, x=x_axis,
                                              y=tx_mcs, label='TX MCS class', color=COLOR_RX_MCS)
        self.df.iloc[start_idx:stop_idx].plot(kind='scatter', ax=ax_mcs, x=x_axis,
                                              y=rx_mcs, label='RX MCS class', color=COLOR_TX_MCS,
                                              ylabel=LABEL_MCS, yticks=np.arange(0, 30 + 1, 1))

        ax_mcs.legend(bbox_to_anchor=(1.1, 1.0), loc='upper left')
        # Save the figure
        plt.savefig(self.filename + subtitle + ".png", dpi=MY_DPI)

    def plot_rssi_sum_and_tx_mcs_distance(self, _mac):
        """
        Plots RSSI sum signal level from heard device and TX MCS class as a function of
        distance to starting point.

        param _mac: MAC address of the device heard.

        return: None
        """
        if self.is_base is True:
            return
        fig, ax_rssi = plt.subplots()
        ax_rssi.set_facecolor(COLOR_FACECOLOR)
        ax_mcs = ax_rssi.twinx()
        # Device specific column names:
        rssi_dbm = _mac + ' rssi [dBm]'
        tx_mcs = _mac + ' TX MCS'

        # Determine x-axis scale
        min_distance = int(min(self.df['distance [m]']))
        max_distance = int(max(self.df['distance [m]']))
        interval = (max_distance - min_distance) / 40
        max_distance += int(interval)
        if interval == 0:
            return
        elif interval > 1:
            interval = int(interval)

        # Determine y-axis scale
        min_rssi = self.df[rssi_dbm].min()
        max_rssi = self.df[rssi_dbm].max()

        if min_rssi == max_rssi and min_rssi == -115:
            min_rssi = -120
            max_rssi = -20

        y_interval = (max_rssi - min_rssi) / 10

        if y_interval == 0:
            y_interval = 1
        elif y_interval > 1:
            y_interval = int(y_interval)

        subtitle = '_rssi_and_distance'
        figure_title = self.filename.replace(self.__homedir, '') + subtitle
        self.df.plot(kind='line', ax=ax_rssi, title=figure_title,
                     grid=True, x='distance [m]', y=rssi_dbm,
                     xticks=(np.arange(min_distance, max_distance, interval)),
                     yticks=(np.arange(min_rssi,
                                       max_rssi, y_interval)),
                     color=COLOR_RSSI, lw=2, figsize=FIGSIZE)

        # Label box upper right corner coordinate relative to base plot coordinate
        ax_rssi.legend(bbox_to_anchor=(-0.03, 1.0), loc='upper right')
        # Rotate x-axis tick labels to make them fit better in the picture
        plt.setp(ax_rssi.get_xticklabels(), rotation=30, ha='right')

        self.df.plot(kind='scatter', ax=ax_mcs, grid=True, x='distance [m]', y=tx_mcs,
                     yticks=(np.arange(min(self.df[tx_mcs]), max(self.df[tx_mcs]) + 1, 1)),
                     label='TX MCS class', color=COLOR_TX_MCS, ylabel=LABEL_MCS)
        ax_mcs.legend(bbox_to_anchor=(1.03, 1.0), loc='upper left')
        # Save the figure
        plt.savefig(self.filename + subtitle + ".png", dpi=MY_DPI)

    def plot_rssi_sum_and_tx_mcs_per_distance_traveled(self, _mac):
        """
        Plots RSSI sum signal level from heard device and TX MCS class as a function of
        total/traveled distance (back and forth) from starting point.

        :param _mac: MAC address of the device heard.

        :return: None
        """
        if self.is_base is True:
            return
        fig, ax_rssi = plt.subplots()
        ax_rssi.set_facecolor(COLOR_FACECOLOR)
        ax_mcs = ax_rssi.twinx()
        # Device specific column names:
        rssi_dbm = _mac + ' rssi [dBm]'
        tx_mcs = _mac + ' TX MCS'

        # Determine x-axis scale
        min_distance = int(min(self.df['distance traveled [m]']))
        max_distance = int(max(self.df['distance traveled [m]']))
        interval = (max_distance - min_distance) / 40
        max_distance += interval
        if interval == 0:
            return
        elif interval > 1:
            interval = int(interval)

        # Determine y-axis scale
        min_rssi = self.df[rssi_dbm].min()
        max_rssi = self.df[rssi_dbm].max()

        if min_rssi == max_rssi and min_rssi == -115:
            min_rssi = -120
            max_rssi = -20

        y_interval = (max_rssi - min_rssi) / 10

        if y_interval == 0:
            y_interval = 1
        elif y_interval > 1:
            y_interval = int(y_interval)

        # Define subtitle for image.
        subtitle = '_rssi_and_total_distance'
        figure_title = self.filename.replace(self.__homedir, '') + subtitle
        self.df.plot(ax=ax_rssi, title=figure_title,
                     grid=True, x='distance traveled [m]', y=rssi_dbm,
                     xticks=(np.arange(min_distance, max_distance, interval)),
                     yticks=(np.arange(min_rssi, max_rssi, y_interval)),
                     figsize=FIGSIZE, color=COLOR_RSSI)

        # Set vertical split line for turning point
        ax_rssi.axvline(max(self.df['distance [m]']), color=COLOR_TURNING_POINT, linestyle='--',
                        label='Turning point', lw=3)

        # Label box upper right corner coordinate relative to base plot coordinate
        ax_rssi.legend(bbox_to_anchor=(-0.03, 1.0), loc='upper right')

        # Rotate x-axis tick labels to make them fit better in the picture
        plt.setp(ax_rssi.get_xticklabels(), rotation=30, ha='right')

        self.df.plot(kind='scatter', ax=ax_mcs, grid=True, x='distance traveled [m]', y=tx_mcs,
                     yticks=(np.arange(min(self.df[tx_mcs]), max(self.df[tx_mcs]) + 1, 1)),
                     label='TX MCS class', color=COLOR_TX_MCS)
        # Label location
        ax_mcs.legend(bbox_to_anchor=(1.03, 1.0), loc='upper left')
        # Save the figure
        plt.savefig(self.filename + subtitle + ".png", dpi=MY_DPI)

    def plot_rssi_and_mcs_per_time(self, _mac):
        """
        Plots RSSI sum signal level as well as antenna specific RSSI
        values, from heard device, and also RX MCS class as a function
        of time.

        :param _mac: MAC address of the device heard.

        :return: None
        """
        fig, ax_rssi = plt.subplots()
        ax_rssi.set_facecolor(COLOR_FACECOLOR)
        ax_mcs = ax_rssi.twinx()

        x_axis = 'Timestamp'
        # Determine x-axis scale
        min_time = int(min(self.df[x_axis]))
        max_time = int(max(self.df[x_axis]))
        x_interval = (max_time - min_time) / 40
        max_time += x_interval
        if x_interval == 0:
            return  # Exit if invalid data
        elif x_interval > 1:
            x_interval = int(x_interval)

        # Device specific column names:
        rssi_dbm = _mac + ' rssi [dBm]'
        rssi_ant0_dbm = _mac + ' rssi ant0 [dBm]'
        rssi_ant1_dbm = _mac + ' rssi ant1 [dBm]'
        rssi_ant2_dbm = _mac + ' rssi ant2 [dBm]'
        rx_mcs = _mac + ' RX MCS'

        # Determine y-axis scale
        min_rssi = pd.to_numeric(self.df[[rssi_dbm,
                                          rssi_ant0_dbm,
                                          rssi_ant1_dbm,
                                          rssi_ant2_dbm]].min().min())
        max_rssi = pd.to_numeric(self.df[[rssi_dbm,
                                          rssi_ant0_dbm,
                                          rssi_ant1_dbm,
                                          rssi_ant2_dbm]].max().max())
        y_interval = (max_rssi - min_rssi) / 10

        if y_interval == 0:
            return  # Exit if invalid data
        elif y_interval > 1:
            y_interval = int(y_interval)

        subtitle = '_rssi_levels_per_time_' + _mac
        figure_title = self.filename.replace(self.__homedir, '') + subtitle
        self.df.plot(ax=ax_rssi, x=x_axis, y=rssi_ant0_dbm)
        self.df.plot(ax=ax_rssi, x=x_axis, y=rssi_ant1_dbm)
        self.df.plot(ax=ax_rssi, x=x_axis, y=rssi_ant2_dbm)
        self.df.plot(ax=ax_rssi, title=figure_title,
                     grid=True, x=x_axis, y=rssi_dbm,
                     xticks=(np.arange(min_time, max_time, x_interval)),
                     yticks=(np.arange(min_rssi, max_rssi, y_interval)),
                     figsize=FIGSIZE, color=COLOR_RSSI)

        # Label box upper right corner coordinate relative to base plot coordinate
        ax_rssi.legend(bbox_to_anchor=(-0.01, 1.0), loc='upper right')
        # Rotate x-axis tick labels to make them fit better in the picture
        plt.setp(ax_rssi.get_xticklabels(), rotation=30, ha='right')

        self.df.plot(kind='scatter', ax=ax_mcs, grid=True, x='Timestamp', y=rx_mcs,
                     yticks=(np.arange(min(self.df[rx_mcs]), max(self.df[rx_mcs] + 1), 1)),
                     label='RX MCS class', color=COLOR_RX_MCS)
        # Label location
        ax_mcs.legend(bbox_to_anchor=(1.03, 1.0), loc='upper left')
        # Save the figure
        plt.savefig(self.filename + subtitle + ".png", dpi=MY_DPI)

    def plot_throughput_and_mcs_per_distance_traveled(self, _mac):
        """
        Plots RX and TX throughput values as well as MCS class values as
        a function of total distance traveled.

        :param _mac: MAC address of the device heard.

        :return: None
        """
        if self.is_base is True:
            return
        fig, ax_throughput = plt.subplots()
        ax_throughput.set_facecolor(COLOR_FACECOLOR)
        ax_mcs = ax_throughput.twinx()
        rx_mcs = _mac + ' RX MCS'
        tx_mcs = _mac + ' TX MCS'

        # Determine x-axis scale
        min_distance = int(min(self.df['distance traveled [m]']))
        max_distance = int(max(self.df['distance traveled [m]']))
        x_interval = (max_distance - min_distance) / 40
        max_distance += x_interval
        if x_interval == 0:
            return
        elif x_interval > 1:
            x_interval = int(x_interval)

        # Determine y-axis scale
        if min(self.df['TX throughput [Bits/s]']) < min(self.df['RX throughput [Bits/s]']):
            min_bitrate = int(min(self.df['TX throughput [Bits/s]']))
        else:
            min_bitrate = int(min(self.df['RX throughput [Bits/s]']))
        if max(self.df['TX throughput [Bits/s]']) > max(self.df['RX throughput [Bits/s]']):
            max_bitrate = int(max(self.df['TX throughput [Bits/s]']))
        else:
            max_bitrate = int(max(self.df['RX throughput [Bits/s]']))
        y_interval = (max_bitrate - min_bitrate) / 10

        if y_interval == 0:
            y_interval = 1
        elif y_interval > 1:
            y_interval = int(y_interval)
        max_bitrate += y_interval

        # Set throughput label names
        rx_throughput_label = 'RX throughput [bit/s]'
        tx_throughput_label = 'TX throughput [bit/s]'
        if self.throughput_units == 'Kb':
            rx_throughput_label = 'RX throughput [Kb/s]'
            tx_throughput_label = 'TX throughput [Kb/s]'
        elif self.throughput_units == 'Mb':
            rx_throughput_label = 'RX throughput [Mb/s]'
            tx_throughput_label = 'TX throughput [Mb/s]'

        subtitle = '_throughput_per_distance_traveled'
        figure_title = self.filename.replace(self.__homedir, '') + subtitle

        self.df.plot(ax=ax_throughput, x='distance traveled [m]', y='RX throughput [Bits/s]',
                     xticks=(np.arange(min_distance, max_distance, x_interval)),
                     yticks=(np.arange(min_bitrate, max_bitrate, y_interval)),
                     label=rx_throughput_label, color=COLOR_RX_THROUGHPUT, lw=2)
        self.df.plot(ax=ax_throughput, title=figure_title,
                     grid=True, x='distance traveled [m]', y='TX throughput [Bits/s]',
                     xticks=(np.arange(min_distance, max_distance, x_interval)),
                     yticks=(np.arange(min_bitrate, max_bitrate, y_interval)),
                     label=tx_throughput_label, color=COLOR_TX_THROUGHPUT, lw=2,
                     figsize=FIGSIZE)

        # Set vertical split line for turning point
        ax_throughput.axvline(max(self.df['distance [m]']), color=COLOR_TURNING_POINT,
                              linestyle='--', label='Turning point', lw=3)

        # Label box upper right corner coordinate relative to base plot coordinate
        ax_throughput.legend(bbox_to_anchor=(-0.03, 1.0), loc='upper right')

        # Rotate x-axis tick labels to make them fit better in the picture
        plt.setp(ax_throughput.get_xticklabels(), rotation=30, ha='right')
        # Determine 2nd y-axis scale
        if min(self.df[tx_mcs]) < min(self.df[rx_mcs]):
            min_mcs = int(min(self.df[tx_mcs]))
        else:
            min_mcs = int(min(self.df[rx_mcs]))
        if max(self.df[tx_mcs]) > max(self.df[rx_mcs]):
            max_mcs = int(max(self.df[tx_mcs]))
        else:
            max_mcs = int(max(self.df[rx_mcs]))
        y_interval = (max_mcs - min_mcs) / 10
        max_mcs += y_interval

        if y_interval == 0:
            return
        elif y_interval > 1:
            y_interval = int(y_interval)

        self.df.plot(kind='scatter', ax=ax_mcs, grid=True, x='distance traveled [m]', y=rx_mcs,
                     yticks=(np.arange(min_mcs, max_mcs, y_interval)),
                     label='RX MCS class', color=COLOR_RX_MCS)
        self.df.plot(kind='scatter', ax=ax_mcs, grid=True, x='distance traveled [m]', y=tx_mcs,
                     yticks=(np.arange(min_mcs, max_mcs, y_interval)),
                     label='TX MCS class', ylabel=LABEL_MCS, color=COLOR_TX_MCS)

        ax_mcs.legend(bbox_to_anchor=(1.03, 1.0), loc='upper left')
        # Save the figure
        plt.savefig(self.filename + subtitle + ".png", dpi=MY_DPI)

    def plot_rx_throughput_and_mcs_per_time(self, _mac):
        """
        Plots RX and TX throughput values as well as MCS class values as
        a function of total distance traveled.

        :param _mac: MAC address of the device heard.

        :return: None
        """
        fig, ax_throughput = plt.subplots()
        ax_throughput.set_facecolor(COLOR_FACECOLOR)
        ax_mcs = ax_throughput.twinx()
        rx_mcs = _mac + ' RX MCS'

        # Determine x-axis scale
        min_timestamp = int(min(self.df['Timestamp']))
        max_timestamp = int(max(self.df['Timestamp']))
        x_interval = (max_timestamp - min_timestamp) / 40

        if x_interval > 10:
            x_interval = round(x_interval / 10) * 10
        elif x_interval == 0:
            x_interval = 1
        max_timestamp += x_interval

        # Determine y-axis scale
        min_bitrate = int(min(self.df['RX throughput [Bits/s]']))
        max_bitrate = int(max(self.df['RX throughput [Bits/s]']))
        y_interval = (max_bitrate - min_bitrate) / 10

        if y_interval == 0:
            y_interval = 1
        elif y_interval > 1:
            y_interval = int(y_interval)
        max_bitrate += y_interval

        subtitle = '_rx_throughput_per_time'
        figure_title = self.filename.replace(self.__homedir, '') + subtitle

        self.df.plot(ax=ax_throughput, title=figure_title,
                     grid=True, x='Timestamp', y='RX throughput [Bits/s]',
                     xticks=(np.arange(min_timestamp, max_timestamp, x_interval)),
                     yticks=(np.arange(min_bitrate, max_bitrate, y_interval)),
                     label='RX throughput [bit/s]', color=COLOR_RX_THROUGHPUT, lw=2,
                     figsize=FIGSIZE)
        # Label box upper right corner coordinate relative to base plot coordinate
        ax_throughput.legend(bbox_to_anchor=(-0.03, 1.0), loc='upper right')
        # Rotate x-axis tick labels to make them fit better in the picture
        plt.setp(ax_throughput.get_xticklabels(), rotation=30, ha='right')

        logger.debug('min rx mcs: %s, max rx mcs: %s', min(self.df[rx_mcs]), max(self.df[rx_mcs]))
        self.df.plot(kind='scatter', ax=ax_mcs, grid=True, x='Timestamp', y=rx_mcs,
                     yticks=(np.arange(min(self.df[rx_mcs]), max(self.df[rx_mcs]) + 1, 1)),
                     label='RX MCS class', color=COLOR_RX_MCS, ylabel=LABEL_MCS)
        # Legend location
        ax_mcs.legend(bbox_to_anchor=(1.03, 1.0), loc='upper left')
        # Save the figure
        plt.savefig(self.filename + subtitle + ".png", dpi=MY_DPI)

    def create_map(self, coordinates):
        """
        Creates a map with travel route including start and stop markers
        and saves output as an HTML file.

        :param coordinates: list of latitude and longitude pairs.

        :return: None
        """
        # Define center point for the map.
        lat = (self.df['latitude'].min() + self.df['latitude'].max()) / 2
        lon = (self.df['longitude'].min() + self.df['longitude'].max()) / 2
        center = (lat, lon)

        # Create a map object:
        m = Map(center=center, zoom=16, layout=Layout(width='100%', height='1300px'), max_zoom=24,
                scroll_wheel_zoom=True)

        # Marker for base device
        base_icon = AwesomeIcon(name='home', icon_color='antiquewhite')
        base_marker = Marker(location=(self.__base_latitude, self.__base_longitude),
                             title='Base', icon=base_icon)
        m.add_layer(base_marker)

        # Starting point for scooter/drone
        start_icon = AwesomeIcon(name='flag', marker_color='green', icon_color='antiquewhite')
        start_marker = Marker(location=coordinates[0], title='Start', icon=start_icon)
        m.add_layer(start_marker)

        # Endpoint for scooter/drone
        end_icon = AwesomeIcon(name='flag-checkered', marker_color='red',
                               icon_color='antiquewhite')
        end_marker = Marker(location=coordinates[len(coordinates) - 1], title='End', icon=end_icon)
        m.add_layer(end_marker)
        # AntPath to display the direction of the coordinates:
        ant_path = AntPath(
            locations=coordinates,
            dash_array=[1, 10],
            delay=1000,
            color='#7590ba',
            pulse_color='#3f6fba'
        )
        # Add the AntPath layer:
        m.add_layer(ant_path)

        # Create measure control
        measure = MeasureControl(position='topleft', active_color='red',
                                 primary_length_unit='meters', primary_area_unit='sqmeters')
        measure.completed_color = 'red'
        # Add control to map
        m.add_control(measure)

        # Save map as HTML file.
        m.save(self.filename + '.html')


def show_usage():
    print('''usage: <file_or_directory_path> [-i|-d|-s|-b<val>|-u] [--log <level>,\
--xaxis <val>, --xstart <val>,\
--xstop <val>, --ismoving <val>]
                    -i: set log level to INFO
                    -d: set log level to DEBUG
                    -s: shows generated plots on UI.
                    -b: defines whether baseband related (temperature, voltage, current) plots
                        are generated. Possible values: 0, 1, false, true. Default value: true.
                    -u: define units for throughput values (b, Kb, Mb)
                    --log <level> where <level> can be INFO or DEBUG
                    --xaxis <val> defines x-axis for plots. Possible values: time, distance, 
                      total_distance. Default value: time.
                    --xstart <val> defines starting point of plot slice.
                    --xstop <val> defines end point of plot slice.
                      For example: --xaxis time --xstart 420 --xstop 840 will produce a figure that
                      contains rssi, noise, throughput and mcs values starting from timestamp value 
                      420s to 840s.
                    --ismoving <val> defines whether or not logged device should be treated as 
                      stationary or moving device. Should be set to zero or false in case tested
                      device has been stationary in actual use case but logging has been on so long
                      time that it cannot be determined from gps coordinates. Default value: true.
                      ''')


if __name__ == '__main__':
    path = ''
    if len(sys.argv) > 1:
        path = sys.argv[1]
    csv_files = []
    show_on_ui = False  # Do not plot on UI by default
    plot_baseband = True
    x_ax = 'Timestamp'
    is_stationary = False
    x_start_val = None
    x_stop_val = None
    tp_unit = 'b'

    if os.path.isdir(path):
        os.chdir(path)
        subdirs = [directory[0] for directory in os.walk(path)]
        for subdir in subdirs:
            files = os.walk(subdir).__next__()[2]
            if len(files) > 0:
                for file in files:
                    if file.endswith(".csv"):
                        csv_files.append(os.path.join(subdir, file))
    else:
        if path.endswith(".csv"):
            csv_files.append(path)

    if not csv_files:
        print('CSV file not found.')
        show_usage()

    # Get optional parameters
    try:
        options, args = getopt.getopt(sys.argv[2:], 'idsb:u:', ['log=', 'xaxis=',
                                                                'xstart=', 'xstop=', 'ismoving='])
    except getopt.error as msg:
        sys.stdout = sys.stderr
        print(msg)
        show_usage()
        sys.exit(2)

    # Define settings based on given configuration options.
    for o, a in options:
        if o == '-i':
            logger.setLevel(logging.INFO)
        if o == '-d':
            logger.setLevel(logging.DEBUG)
        if o == '-s':
            show_on_ui = True
        if o == '--log':
            if a.upper() == 'INFO':
                logger.setLevel(logging.INFO)
            if a.upper() == 'DEBUG':
                logger.setLevel(logging.DEBUG)
        if o == '-u':
            if a == 'b':
                tp_unit = 'b'
                logger.info('Setting throughput units to bits per seconds.')
            if a == 'Kb':
                tp_unit = 'Kb'
                logger.info('Setting throughput units to kilobits per seconds.')
            if a == 'Mb':
                tp_unit = 'Mb'
                logger.info('Setting throughput units to megabits per seconds.')
        if o == '-b':
            if a.lower() == '0' or a.lower() == 'false':
                plot_baseband = False
        if o == '--xaxis':
            if a.lower() == 'time':
                logger.info('Setting x axis to time.')
                x_ax = 'Timestamp'
            if a.lower() == 'distance':
                logger.info('Setting x axis to distance.')
                x_ax = 'distance [m]'
            if a.lower() == 'total_distance':
                logger.info('Setting x axis to total_distance.')
                x_ax = 'distance traveled [m]'
        if o == '--xstart':
            x_start_val = a
        if o == '--xstop':
            x_stop_val = a
        if o == '--ismoving':
            if a.lower() == '0' or a.lower() == 'false':
                is_stationary = True

    for file in csv_files:
        print(file)
        try:
            ftlp = FieldTestLogPlotter(file, tp_unit)
            if plot_baseband:
                ftlp.plot_temp_voltage_and_current()
                ftlp.plot_temperature_and_current()
                ftlp.plot_voltage_and_current()
            ftlp.create_map(ftlp.coordinates)
            for mac in ftlp.mac_list:
                if mac == '00:00:00:00:00:00':
                    continue
                ftlp.rssi_noise_throughput_and_mcs(mac, x_ax, x_start_val, x_stop_val)
                if not ftlp.is_base and not is_stationary:
                    if x_ax is not None:
                        ftlp.rssi_noise_throughput_and_mcs(mac, 'distance traveled [m]',
                                                           x_start_val, x_stop_val)
                    ftlp.plot_rssi_sum_and_tx_mcs_distance(mac)
                    ftlp.plot_rssi_sum_and_tx_mcs_per_distance_traveled(mac)
                    ftlp.plot_throughput_and_mcs_per_distance_traveled(mac)
                ftlp.plot_rx_throughput_and_mcs_per_time(mac)
                ftlp.plot_rssi_and_mcs_per_time(mac)
        except Exception as e:
            print("Could not plot log: ", file)
            print("Reason:", e)
    # Show generated plots
    if show_on_ui:
        show_plots()
