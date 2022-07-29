# Field test log plotter

Field test log plotter is used to generate various graphs from field test logger tool logs. 
Generated graphs are stored as .png/.html files into the same directory where the original field
test log file exists.


## Installation

### Create virtual environment

Run following commands in field_test_log_plotter directory to create and activate virtual
environment.
```
$ python3 -m venv ./venv
$ source ./venv/bin/activate
```

### Install required packages
Install packages listed in requirements.txt into virtual environment.
```
$ pip3 install -r requirements.txt
```

## Usage

Create graphs from all the logs in specified directory and subdirectories:
```
$ python3 ftl_plotter.py ~/logs/indoor_mesh_test_20220608
```
Create graphs from specified log file:
```
$ python3 ftl_plotter.py ~/logs/indoor_mesh_test_20220608/2022-06-08_12:54:03_00:30:1a:4f:5b:0a.csv
```
Additional control options:

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

Create graphs from specified log file between timestamps 420 and 840s. Show generated plots
on UI but do not generate baseband related figures. Use megabits as throughput values:
```
$ python3 ftl_plotter.py ~/logs/indoor_mesh_test_20220608/2022-06-08_12:54:03_00:30:1a:4f:5b:0a.csv --xaxis time --xstart 420 --xstop 840 -s -b 0 -u Mb
```