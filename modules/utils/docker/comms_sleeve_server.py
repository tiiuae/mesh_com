#!/usr/bin/python
import argparse
from flask import Flask
import os as osh
import subprocess

app = Flask(__name__)

def comms_sleeve_get_serial_no():
    cs_serial = "0000000000000000"
    try:
      f = open('/proc/cpuinfo','r')
      for line in f:
        if line[0:6]=='Serial':
          cs_serial = line[10:26]
      f.close()
    except:
      cs_serial = "ERROR000000000"

    return cs_serial

@app.route("/")
def echo_comms_sleeve():
    echo_output = "ERROR00000000"
    if osh.environ.get('HOSTNAME') == "br_hardened":
          cs_serial = comms_sleeve_get_serial_no()
          echo_output = "comms sleeve:" + cs_serial + "\n"
    return echo_output

@app.route("/mesh/stop")
def stop_mesh():
	subprocess.run("/opt/S9011sMesh stop".split())
	return "Stopping comms sleeve mesh service"

@app.route("/mesh/start")
def start_mesh():
	subprocess.run("/opt/S9011sMesh start".split())
	return "Starting comms sleeve mesh service"

@app.route("/mesh_profile/low_latency")
def enable_low_latency_config():
	return "comms sleeve low latency config enabled!"

@app.route("/mesh_profile/long_range")
def enable_long_range_config():
	return "comms long range config enabled!"


@app.route("/mesh_profile/performance")
def enable_perf_config():
	return "comms sleeve performance config enabled"

@app.route("/comms_sleeve_shutdown")
def shutdown():
	subprocess.run("shutdown -h now".split())
	return "Shutting down comms_sleeve"

@app.route("/comms_sleeve_reboot")
def reboot():
	subprocess.run("shutdown -r now".split())
	return "Restarting comms sleeve"

if __name__ == '__main__':

    # Construct the argument parser
    comms_sleeve_cfg = argparse.ArgumentParser()

    # Add the arguments to the parser
    comms_sleeve_cfg.add_argument("-ip", "--ip_address", required=True, help="IP address of the mesh bridge(br-lan) interface")
    comms_sleeve_cfg.add_argument("-ap_if", "--ap_interface", required=True)
    comms_sleeve_cfg.add_argument("-mesh_if", "--mesh_interface", required=True)
    args = comms_sleeve_cfg.parse_args()

    app.run(host=args.ip_address, port=5000)
