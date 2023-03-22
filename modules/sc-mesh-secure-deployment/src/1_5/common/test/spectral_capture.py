import sys
import os
import yaml
import subprocess
import pandas as pd
sys.path.append('../')
from SpectralMgr import Spectral

if __name__ == "__main__":
    count = 0
    subprocess.call("rm /tmp/data*", shell=True)

    # SCAN MODE: Check if scan mode arg passed, if not set to default
    try:        
        scan_mode = sys.argv[1].strip()
        if(scan_mode != 'high_latency' and scan_mode != 'low_latency'):
            scan_mode = 'default'      
    except:
        scan_mode = 'default'
    
        
    # FREQUENCY BAND: Check if band arg passed, if not set to all bands   
    try:
        band = sys.argv[2].strip()
        if(scan_mode == 'high_latency' and (band != 'band_2_4' and band != 'band_5')): # if param passed but invalid input
            scan_mode = 'default'    
    except:     
        scan_mode = 'default'
        

    # CONFIG
    with open('spectral_scan_config.yaml') as file:
      try: 
        config = yaml.safe_load(file)
        
        if(scan_mode == 'low_latency'):
            scan_channels = config[scan_mode]['channels'][0]
            all_channels = config[scan_mode]['channels'][0]
        elif(scan_mode == 'high_latency'):
            scan_channels = config[scan_mode][band]['channels']
            all_channels = config[scan_mode][band]['channels']             
        else:
            scan_channels = config['default']['channels']
            all_channels = config['default']['channels']
      except yaml.YAMLError as exc:
        print(exc)


    while (True):
       spec = Spectral()
       spec.initialize_scan()
       spec.execute_scan(scan_channels)
       f = spec.file_open(f"/tmp/data")
       file_stats = os.stat(f"/tmp/data")
       scan_channels = spec.read(f, file_stats.st_size, all_channels, scan_channels, count)
       spec.file_close(f)
       count += 1
