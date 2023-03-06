import sys
import os
import pandas as pd
sys.path.append('../')
from SpectralMgr import Spectral

if __name__ == "__main__":

    spec = Spectral()
    spec.initialize_scan()
    spec.execute_scan()
    f = spec.file_open("/tmp/data")
    file_stats = os.stat("/tmp/data")
    spec.read(f, file_stats.st_size)
    spec.file_close(f)
    
    vals_dict = []
    vals_dict = spec.get_values()

    vals_list = []
    for key, value in vals_dict.items():
       vals_list.append(list(value))


    driver = os.popen('ls /sys/kernel/debug/ieee80211/phy* | grep ath').read().strip()  

    if(driver == "ath9k"): 
      spectral_capture_df = pd.DataFrame(vals_list, columns = ["freq1", "noise", "rssi", "signal", "max magnitude"])
      spectral_capture_df = spectral_capture_df.reindex(columns = ["freq1", "noise","signal", "max_magnitude","total_gain_db", "base_pwr_db", "rssi", "relpwr_db", "avgpwr_db"])

    elif(driver == "ath10k"):
      spectral_capture_df = pd.DataFrame(vals_list, columns = ["freq1", "noise", "max_magnitude", "total_gain_db","base_pwr_db", "rssi", "relpwr_db", "avgpwr_db"])


    spectral_capture_df.to_csv('spectral_scan.csv', index=False)
