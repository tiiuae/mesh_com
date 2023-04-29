import yaml
import sys
import os
import re

# Get mesh interface and current frequency
iw_output = os.popen('iw dev').read()
iw_output = re.sub('\s+', ' ', iw_output).split(' ')

# Parse multiple interfaces
size = len(iw_output)
idx_list = [idx - 1 for idx, val in enumerate(iw_output) if val == "Interface"]
if(len(idx_list) > 1): # if there is more than one interface up, create list for each
    idx_list.pop(0) # print info not needed
iw_interfaces = [iw_output[i: j] for i, j in zip([0] + idx_list, idx_list + ([size] if idx_list[-1] != size else []))]

# Check if mesh interface is up and get interface name and current channel
no_mesh = False
for interface_list in iw_interfaces:  
    try:
        mesh_index = interface_list.index("mesh")
        mesh_list = interface_list
        no_mesh = False
        break
    except:
        no_mesh = True

if(no_mesh):    
    sys.exit("No mesh interface to scan/No mesh set up")

mesh_interface_index = mesh_list.index("Interface") + 1
mesh_interface = mesh_list[mesh_interface_index].split()[0]
channel_index = mesh_list.index("channel") + 2 # index of channel frequency
channel_freq = re.sub("[^0-9]", "", mesh_list[channel_index]).split()[0]

# Get channels available to be scanned by interface
iw_channels = os.popen(f'iwlist {mesh_interface} channel').read()
iw_channels = re.sub('\s+', ' ', iw_channels).split('GHz')

channel_list = []
count = 1
for channel in iw_channels:
    try:
        if(count == 1):
            temp_list = re.sub('\s+', ' ', channel).split(':')
            freq_val = int(float(temp_list[-1].strip())*10**3)
        else:
            freq_val = int(float(channel[channel.index(":")+1:].strip())*10**3)
        
        channel_list.append(freq_val)
        count += 1
    except:
        None

# Temporary workaround for mesh scanning: mesh interface cannot scan it's own channel so remove it temporarily
all_bands = ' '.join(str(ch) for ch in channel_list)
all_bands = all_bands.replace(channel_freq, '').strip()

# Split at first instance of 5GHz channel
for i, num in enumerate(channel_list):
    if str(num).startswith('5'):
        split_index = i
        break

channels_2_4 = channel_list[:split_index]
channels_5 = channel_list[split_index-1:] # include one 2.4 channel to the 5 band as workaround to bug in max_mag range shift

channels_2_4 = ' '.join(str(ch) for ch in channels_2_4)
channels_5 = ' '.join(str(ch) for ch in channels_5)

# remove mesh channel
channels_2_4 = channels_2_4.replace(channel_freq, '').strip()
channels_5 = channels_5.replace(channel_freq, '').strip()

# Spectral scan config
config = {'debug': True,
        'interface': mesh_interface,
        'default': {'channels': all_bands}, #, 'interface': 'wlp1s0'
        'low_latency': {'channels': channel_freq},
        'high_latency': {'band_2_4': {'channels': channels_2_4},
                         'band_5': {'channels': channels_5}}}

# Create YAML for config data
yaml.dump(config, sort_keys=False)
with open('config_spectralscan.yaml', 'w',) as f :
    yaml.dump(config, f, sort_keys=False)
