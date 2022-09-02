# Field test log player

Field test log player is used to replay recorded Node log file.

## Installation

### Create virtual environment

Run following commands in field_test_log_player directory to create and activate virtual
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

Create animated graph from all the logs in specified directory:
```
$ python3 ftl_player.py -f ~/logs/mesh_test
```
Control options:

    -f: Path to folder which includes field_test_logger CSV log files from one specific test run.
        Folder should include one log file from each Node DUT.
    -m: MAC address of a moving node.  This option will add trace from old node positions

Create animated graph from all the logs in specified directory and add trace from moving node:
```
$ python3 ftl_player.py -f ~/logs/mesh_test -m 11:22:33:44:55:66
```
HOX!!

field_test_logger generated log files needs to include GPS timestamp column.  ftl_player will synchronize
log files based on GPS time.
