import yaml
import time
import os
# Import classes for features
# Needs to be replaced by actual features
from features.sample_feature_classes.ids import IDS
from features.sample_feature_classes.SP_CRA import PHYCRA
from features.sample_feature_classes.RSS_auth import RSS_Auth
from features.decision_engine.decision_engine import DecisionEngine

# ************************************************************ Note **************************************************************************
# The MBA module utilizes the certificates exchanged during CBMA to sign and verify messages
# So CBMA must be setup before running this code. Refer to /opt/mesh_com/modules/sc-mesh-secure-deployment/src/2_0/features/cbma/setup_cbma.py
# ********************************************************************************************************************************************

# Defining constants (these can be configured as needed)
MBA_MULTICAST_ADDRESS = 'ff02::1'
MBA_PORT = 12345
MBA_INTERFACE = "bat1"
QUARANTINE_PERIOD = 10 # Quarantine period in seconds
PATH_TO_MY_CERT_DIR = f"{os.path.dirname(__file__)}/features/cbma/cert_generation/certificates" # Path to the folder where private key is stored (Needs to be updated when we use HSM)
PATH_TO_PEER_CERT_DIR = "/tmp/peer_certificates" # Path to the folder where peer certificates are stored by cbma/auth/authClient.py and cbma/auth/authServer.py

def launch_PHY(decision_engine):
    # Place holder to launch PHY
    phycra = PHYCRA(decision_engine)
    phycra.start()
    return phycra

def launch_RSS(decision_engine):
    # Place holder to launch RSS
    rss_authen = RSS_Auth(decision_engine)
    rss_authen.start()
    return rss_authen

def launch_IDS(decision_engine):
    # Place holder to launch IDS
    ids = IDS(decision_engine)
    ids.start()
    return ids

def launch_jamming():
    # Place holder to launch jamming
    # Currently, this function does nothing
    pass

def stop(sensors, decision_engine):
    for sensor in sensors.values():
        sensor.stop()
    decision_engine.stop()

def readfile():
    with open("features.yaml", "r") as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return None

def initialize(feature, decision_engine):
    if feature == 'PHY':
        phycra = launch_PHY(decision_engine)
        sensors[feature] = phycra
    if feature == 'RSS':
        rss_authen = launch_RSS(decision_engine)
        sensors[feature] = rss_authen
    if feature == 'IDS':
        ids = launch_IDS(decision_engine)
        sensors[feature] = ids
    if feature == 'jamming':
        jamming = launch_jamming()
        sensors[feature] = jamming

if __name__ == "__main__":
    decision_engine = DecisionEngine(mba_multicast_group=MBA_MULTICAST_ADDRESS, mba_port=MBA_PORT, mba_interface=MBA_INTERFACE, quarantine_period=QUARANTINE_PERIOD, my_cert_dir=PATH_TO_MY_CERT_DIR, peer_cert_dir=PATH_TO_PEER_CERT_DIR)
    features = readfile()
    sensors = {} # Stores sensor_name: sensor_object for each sensor/ feature that publishes results for decision engine

    # Start features that are true in features.yaml
    for feature in features:
        if features[feature]:
            initialize(feature, decision_engine)
    time.sleep(15)
    stop(sensors, decision_engine)