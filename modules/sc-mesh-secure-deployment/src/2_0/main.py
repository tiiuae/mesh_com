import yaml
import threading
import time

# Import classes for features
from features.PHY.PHY_CRA_scripts.SP_CRA_mainDE1 import PHYCRA
from features.PHY.RSS_auth.F_RSS_Auth import RSS_Auth
# from features.IDS.IDS import IDS
# from features.jamming.jamming import Jamming

def launch_PHY():
    # Place holder to launch PHY
    phycra = PHYCRA()
    phycra.start()
    return phycra

def launch_RSS():
    # Place holder to launch RSS
    rss_authen = RSS_Auth()
    rss_authen.start()
    return rss_authen

def launch_IDS():
    # Place holder to launch IDS
    # Currently, this function does nothing
    pass

def launch_jamming():
    # Place holder to launch jamming
    # Currently, this function does nothing
    pass

def stop_PHY(phycra):
    phycra.stop()

def stop_RSS(rss_authn):
    rss_authn.stop()

def stop_jamming(jamming):
    jamming.stop()

def stop_IDS(ids):
    ids.stop()

def launch_decision_engine(sensors):
    # Place holder to launch decision engine
    collected_data = {}
    # decision_engine = DecisionEngine(sensors)

    # Periodically gets results from security sensors
    while True:
        print("Executing decision engine with collected data: ")
        for sensor_name in sensors:
            sensor_data = sensors[sensor_name].get_result()
            if sensor_data:  # Check if there is new data
                collected_data[sensor_name] = sensor_data
            else:
                collected_data[sensor_name] = {'Pass': [], 'Fail': []}  # Default to empty lists if no new data

        for sensor_name in collected_data:
            print(sensor_name)
            print(collected_data)


            # Process data with the decision engine
            #decisions = decision_engine.make_decision(collected_data)

            # Place holder to call quarantine/ MBA if necessary

        time.sleep(40) # Period can be adjusted



def readfile():
    with open("features.yaml", "r") as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return None

def initialize(feature):
    if feature == 'PHY':
        phycra = launch_PHY()
        sensors[feature] = phycra
    if feature == 'RSS':
        rss_authen = launch_RSS()
        sensors[feature] = rss_authen
    if feature == 'IDS':
        ids = launch_IDS()
        sensors[feature] = ids
    if feature == 'jamming':
        launch_jamming()

if __name__ == "__main__":
    features = readfile()
    sensors = {} # Stores sensor_name: sensor_object for each sensor/ feature that publishes results for decision engine

    # Start features that are true in features.yaml
    for feature in features:
        if features[feature]:
            initialize(feature)

    # Call decision engine
    launch_decision_engine(sensors)
