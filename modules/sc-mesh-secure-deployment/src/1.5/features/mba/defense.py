#!/usr/bin/python

# import comms
import subprocess
import paho.mqtt.client as mqtt
import threading
import yaml
import os
import netifaces
from watchdog.observers import Observer
from watchdog.events import PatternMatchingEventHandler
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from pathlib import Path

TOPIC = 'malicious/mac'

# pub = comms.Announce()
# mqttc = comms.MyMQTTClass()
globalMessagePayload = ''  # HERE!
blackList = []

confPath = '~/mesh_com/modules/sc-mesh-secure-deployment/src/mesh_com.conf'
PATH = '/tmp/a.txt'

if os.path.exists(confPath):  # verify conf file
    with open(confPath, 'r') as stream:
        try:
            parsed_yaml_file = yaml.load(stream, Loader=yaml.FullLoader)
            if "mesh_inf" in parsed_yaml_file["client"]:
                interface = parsed_yaml_file['client']['mesh_inf']
            else:
                interface_list = netifaces.interfaces()
                interface = filter(lambda x: 'wlx' in x, interface_list)[0]
        except yaml.YAMLError as exc:
            print(exc)


class FileMonitoring(FileSystemEventHandler):  # monitoring non-auth file
    def __init__(self):
        if PATH.is_file():
            with open(PATH, 'r') as fi:
                aux = fi.readlines()
                publisher(get_broker_ip(), aux[-1])

    def on_modified(self, event):
        print('File Modified ' + str({event.src_path}))
        with open(PATH, 'r') as fi:
            aux = fi.readlines()
            publisher(get_broker_ip(), aux[-1])


def monitoring():
    event_handler = FileMonitoring()
    observer = Observer()
    observer.schedule(event_handler, path=PATH, recursive=False)
    observer.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()


def publisher(host, message):
    client = mqtt.Client()
    client.connect(host, 1883, 60)
    client.publish(TOPIC, message, retain=True)
    client.disconnect()


def get_broker_ip():
    cmd1 = "avahi-browse -rptf _mqtt._tcp"
    broker = subprocess.check_output(cmd1, shell=True)
    if broker:
        ip = broker.decode().split('\n=')[-1].split(';')[-3]
        print('Broker on IP: ', ip)
        return ip
    else:
        print('No Broker found on the network')


def on_message(client, userdata, msg):  # call_back
    print('here')
    global blackList
    global globalMessagePayload
    globalMessagePayload = msg.payload.decode()  # HERE!
    if globalMessagePayload and globalMessagePayload not in blackList:
        blackList.append(globalMessagePayload)
        block(globalMessagePayload)
    print(msg.topic + " " + str(msg.payload))


def on_connect(client, userdata, flags, rc):  # call_back
    print("Connected with result code " + str(rc))
    client.subscribe(TOPIC)


def subscribe(ip):
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(ip, 1883, 60)
    client.loop_forever()


def block(mac):
    subprocess.Popen(['traffic_block.sh', mac, interface], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    print('blocking')


def start_thread():
    # Process creation
    p = threading.Thread(target=subscribe, args=(get_broker_ip(),))
    p.start()
    p2 = threading.Thread(target=monitoring)
    p2.start()
    # p.join()


if __name__ == '__main__':
    start_thread()


