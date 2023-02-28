import yaml
from main_with_menu import *

MA_thread = None
def readfile():
    with open("features.yaml", "r") as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)


def initialize(feature):
    global MA_thread
    if feature == 'mutual':
            MA_thread = MA()
    if feature == 'continuous':
            CA()
    if feature == 'NESS':
            DE()
    if feature == 'secbeat':
            sbeat_client()
    if feature == 'quarantine':
            Quarantine()
    if feature == 'only_mesh':
            only_mesh()



if __name__ == "__main__":
    threadList = []
    features = readfile()
    for index in features:
        if features[index]:
            initialize(index)
    # wait for Auth_AP to start in background for future nodes
    if MA_thread:
        MA_thread.join()
