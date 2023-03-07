# Field test Logger tool

Field test logger tool is used to collect various Radio Wi-Fi, environment, location and Mesh 
networking related data items. 
Generated log file is stored in .csv file into /root/field_test_logs/ - folder in the device.


## Installation

### Copy files

Copy files to correct folders.  Replace <ip> with your device IP address, if you are using below
commands for copy procedure:

```
$ scp S95logger root@<ip>:/etc/init.d/
$ scp *.py root@<ip>:/usr/local/bin/
```

### Check permissions

After copying the files it is recommended to check file permissions.

```
$ chmod +x /etc/init.d/S95logger
$ chmod +x /usr/local/bin/field_test_logger.py
$ chmod +x /usr/local/bin/gpsd.py
$ chmod +x /usr/local/bin/wifi_info.py
$ chmod +x /usr/local/bin/infoparser.py
```

## Usage
Logger is started automatically in every device boot-up.
Log files are generated to /root/field_test_logs/ - folder.

Default log file name format is: YYYY-MM-DD_HH:MM:SS_<WIFI_MAC>.csv

Example: 1970-01-01_00:00:13_00:30:1a:4f:17:74.csv

It is also possible to define use case specific suffix for file name via 
command line argument. As there can be only one instance of field test logger running
one needs to stop it before starting a new one with command line argument.

```
$ /etc/init.d/S95logger stop
$ /etc/init.d/S95logger start my_test_case1
```
Now filename would be: 1970-01-01_00:00:13_00:30:1a:4f:17:74_my_test_case1.csv


