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
