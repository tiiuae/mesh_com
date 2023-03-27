import yaml
import sys
import os
import re


# Get mesh interface and frequency channel being used
iw_output = os.popen('iw dev').read()
iw_output = re.sub('\s+', ' ', iw_output).split(' ')
type_index = iw_output.index("type") + 1


if (iw_output[type_index] == "mesh"):
    mesh_interface_index = iw_output.index("Interface") + 1
    mesh_interface = iw_output[mesh_interface_index].split()[0]
    channel_index = iw_output.index("channel") + 2 # index of channel frequency
    channel_freq = re.sub("[^0-9]", "", iw_output[channel_index]).split()[0]
else:
    sys.exit("No mesh interface to scan")


# Workaround for mesh freq scanning issue: mesh interface cannot scan it's own channel freq, so remove it temporarily
all_bands = '2412 2417 2422 2427 2432 2437 2442 2447 2452 2457 2462 5180 5200 5220 5240 5260 5280 5300 5320 5745 5765 5785 5805 5825'
all_2_4 = '2412 2417 2422 2427 2432 2437 2442 2447 2452 2457 2462'
all_5 = '5180 5200 5220 5240 5260 5280 5300 5320 5745 5765 5785 5805 5825'

all_bands = all_bands.replace(channel_freq, '').strip()
all_2_4 = all_2_4.replace(channel_freq, '').strip()
all_5 = all_5.replace(channel_freq, '').strip()


# Spectral scan config
config = {'debug': True,
        'interface': mesh_interface,
        'default': {'channels': all_bands, 'interface': 'wlp1s0'},
        'low_latency': {'channels': channel_freq},
        'high_latency': {'band_2_4': {'channels': all_2_4},
                         'band_5': {'channels': all_5}}}


# Create YAML for config dict data
yaml.dump(config, sort_keys=False)
with open('spectral_scan_config.yaml', 'w',) as f :
    yaml.dump(config, f, sort_keys=False)

