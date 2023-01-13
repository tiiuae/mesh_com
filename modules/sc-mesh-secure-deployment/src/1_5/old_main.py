import yaml
from main_with_menu import *

def readfile():
    with open("features.yaml", "r") as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)


def initialize(feature):
    if feature == 'mutual':
            MA()
    if feature == 'continuous':
            CA()
    if feature == 'NESS':
            DE()
    if feature == 'secbeat':
            sbeat()
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

