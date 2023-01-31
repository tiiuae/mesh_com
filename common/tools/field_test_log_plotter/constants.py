
# Plot labels
LABEL_TIME_SECONDS = 'Time / Seconds'
LABEL_TEMPERATURE = 'Temperature / Celsius'
LABEL_VOLTAGE = 'Voltage [V]'
LABEL_CURRENT = 'Current [A]'
LABEL_THROUGHPUT = 'Throughput [bit/s]'
LABEL_SIGNAL_STRENGTH = 'Signal strength [dBm]'
LABEL_SNR = 'SNR [dB]'
LABEL_MCS = 'MCS class'

OUT_OF_SCALE_RSSI = -115
# Plot y-axis ranges
"""
TEMP_RANGE = (0, 100)
VOLTAGE_RANGE = (3, 4.3)  # 3 - 4.3
CURRENT_RANGE = (0, 2.5)  # 0 - 1.5
SIGNAL_STRENGTH_RANGE = (-100, 0)
MCS_RANGE = (0, 30)  # Modulation Coding Scheme?
SNR_RANGE = (0, 100)
"""

# 'Final' dataframe column names
TIMESTAMP = 'Timestamp'
SNR = 'snr [dB]'
CPU_TEMP = 'cpu temp [C]'
WIFI_TEMP = 'wifi temp [C]'
TMP100 = 'tmp100 [C]'
BATT_TEMP = 'battery temp [C]'
BATT_VOLTAGE = 'battery voltage [V]'
BATT_CURRENT = 'battery current [A]'
NRF_VOLTAGE = 'nRF voltage [V]'
NRF_CURRENT = 'nRF current [A]'
MPCIE_VOLTAGE = 'mPCIe voltage [V]'
MPCIE_CURRENT = 'mPCIe current [A]'
DCIN_VOLTAGE = 'DCin voltage [V]'
DCIN_CURRENT = 'DCin current [A]'

# Fixed color codes for plots.
# Ref. https://datascientyst.com/full-list-named-colors-pandas-python-matplotlib/
COLOR_FACECOLOR = 'snow'  # Good alternatives e.g. whitesmoke, snow, white
COLOR_TURNING_POINT = 'lightcoral'

COLOR_CPU_TEMP = 'darkorange'
COLOR_WIFI_TEMP = 'palegreen'
COLOR_TMP100 = 'deeppink'
COLOR_BATT_TEMP = 'teal'

COLOR_BATT_VOLTAGE = 'green'
COLOR_RF_VOLTAGE = 'plum'
COLOR_3V3_VOLTAGE = 'crimson'
COLOR_DCIN_VOLTAGE = 'goldenrod'

COLOR_BATT_CURRENT = 'fuchsia'
COLOR_RF_CURRENT = 'darksalmon'
COLOR_3V3_CURRENT = 'blue'
COLOR_DCIN_CURRENT = 'cadetblue'

COLOR_RSSI = 'black'  # Was crimson, cornflowerblue
COLOR_NOISE = 'red'  # Was dodgerblue

COLOR_RX_THROUGHPUT = 'mediumseagreen'  # Was goldenrod
COLOR_TX_THROUGHPUT = 'sienna'

COLOR_RX_MCS = 'royalblue'
COLOR_TX_MCS = 'red'

# Figure size targeted for 32" 16:9 display
FIGSIZE = (27.89, 15.69)
MY_DPI = 91.789171746
