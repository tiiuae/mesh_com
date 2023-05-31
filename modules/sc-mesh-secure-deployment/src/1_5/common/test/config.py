import yaml
import sys
import os
import re

scan_interface_type = "mesh"

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
no_interface = False
for interface_list in iw_interfaces:  
    try:
        interface_index = interface_list.index(scan_interface_type) # change to the type of the interface used to scan
        interface_list = interface_list
        no_interface = False
        break
    except:
        no_interface = True

if(no_interface):    
    sys.exit(f"No {scan_interface_type} interface to scan")

interface_index = interface_list.index("Interface") + 1
interface = interface_list[interface_index].split()[0]
try:
   channel_index = interface_list.index("channel") + 2 # index of channel frequency
   channel_freq = re.sub("[^0-9]", "", interface_list[channel_index]).split()[0]
except: 
   channel_freq = "" # interface is not set to any channel

# Get channels available to be scanned by interface
iw_channels = os.popen(f'iwlist {interface} channel').read()
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
        'interface': interface, 
        'default': {'channels': all_bands},
        'low_latency': {'channels': channel_freq},
        'high_latency': {'band_2_4': {'channels': channels_2_4},
                         'band_5': {'channels': channels_5}}}

# Create YAML for config data
yaml.dump(config, sort_keys=False)
with open('config.yaml', 'w',) as f :
    yaml.dump(config, f, sort_keys=False)
