import yaml
import sys
import os
import re


# Get mesh interface and frequency channel being used
iw_output = os.popen('iw dev').read()
iw_output = re.sub('\s+', ' ', iw_output).split(' ')


# Split list for multiple interfaces
size = len(iw_output)
idx_list = [idx - 1 for idx, val in enumerate(iw_output) if val == "Interface"]
if(len(idx_list) > 1): # if there is more than one interface up, create list for each
    idx_list.pop(0)
iw_interfaces = [iw_output[i: j] for i, j in zip([0] + idx_list, idx_list + ([size] if idx_list[-1] != size else []))]


# Check if mesh interface is up, if not exit
no_mesh = False
for interface_list in iw_interfaces:  
    try:
        mesh_index = interface_list.index("mesh")
        mesh_list = interface_list
        no_mesh = False
    except:
        no_mesh = True

if(no_mesh):    
    sys.exit("No mesh interface to scan/No mesh set up")
    

mesh_interface_index = mesh_list.index("Interface") + 1
mesh_interface = mesh_list[mesh_interface_index].split()[0]
channel_index = mesh_list.index("channel") + 2 # index of channel frequency
channel_freq = re.sub("[^0-9]", "", mesh_list[channel_index]).split()[0]


# Temporary workaround for mesh scanning: mesh interface cannot scan it's own channel so remove it temporarily
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


# Create YAML for block of data
yaml.dump(config, sort_keys=False)
with open('config_spectralscan.yaml', 'w',) as f :
    yaml.dump(config, f, sort_keys=False)

