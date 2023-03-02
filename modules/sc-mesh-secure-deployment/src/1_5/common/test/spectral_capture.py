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
    spec.read(f, 8512)
    spec.file_close(f)

    vals_dict = []
    vals_dict = spec.get_values()

    vals_list = []
    for key, value in vals_dict.items():
          vals_list.append(list(value))


    driver = os.popen('ls /sys/kernel/debug/ieee80211/phy* | grep ath').read().strip()

    if(driver == "ath9k"):
        spectral_capture_df = pd.DataFrame(vals_list, columns = ["tsf", "freq", "noise", "rssi", "max_magnitude"])
        spectral_capture_df = spectral_capture_df.reindex(columns = ["freq1", "freq2", "noise", "max_magnitude",
                                                                 "total_gain_db", "base_pwr_db", "rssi", "relpwr_db", "avgpwr_db", "tsf"])

    elif(driver == "ath10k"):
        #spectral_capture_df = pd.DataFrame(vals_list, columns = ["freq1", "freq2", "noise", "max_magnitude",
        #                                                         "total_gain_db", "base_pwr_db", "rssi", "relpwr_db", "avgpwr_db", "tsf"])
        spectral_capture_df = pd.DataFrame(vals_list, columns = ["tsf", "freq", "noise", "rssi", "max_magnitude"])

    spectral_capture_df.to_csv('spectral_scan.csv')
    print(spectral_capture_df)
