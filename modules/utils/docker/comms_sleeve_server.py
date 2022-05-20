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

def disable_wifi_powersave():
    cmd = "iw dev " + str(mesh_if) + " set power_save off"
    print(cmd)
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)

def enable_wifi_powersave():
    cmd = "iw dev " + str(mesh_if) + " set power_save on"
    print(cmd)
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)

def config_amsdu_ampdu_aggregation(amsdu, ampdu):
    # echo "amsdu ampdu" > /sys/kernel/debug/ieee80211/phy0/ath10k/htt_max_amsdu_ampdu
    cmd =  "echo " + '"'+ str(amsdu) + " " +  str(ampdu) + '"' + " > /sys/kernel/debug/ieee80211/`iw dev " + str(mesh_if) + " info | gawk '/wiphy/ {printf \"phy\" $2}'`/ath10k/htt_max_amsdu_ampdu"
    print(cmd)
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)

def set_tx_pwr(tx_pwr):
    cmd = "iw phy `iw dev " + str(mesh_if) + " info | gawk '/wiphy/ {printf \"phy\" $2}'` set txpower fixed " + str(tx_pwr)
    print(cmd)
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)

def set_distance(distance):
    cmd = "iw phy `iw dev " + str(mesh_if) + " info | gawk '/wiphy/ {printf \"phy\" $2}'` set distance " + str(distance)
    print(cmd)
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)

def set_coverage_class(coverage_class):
    global mesh_if
    cmd = "iw phy `iw dev " + str(mesh_if) + " info | gawk '/wiphy/ {printf \"phy\" $2}'` set coverage " + str(coverage_class)
    print(cmd)
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)

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
        disable_wifi_powersave()
	return "comms sleeve low latency config enabled!"

@app.route("/mesh_profile/long_range")
def enable_long_range_config():
	return "comms long range config enabled!"


@app.route("/mesh_profile/performance")
def enable_perf_config():
        disable_wifi_powersave()
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
    ap_if = args.ap_interface
    mesh_if = args.mesh_interface
    app.run(host=args.ip_address, port=5000)
