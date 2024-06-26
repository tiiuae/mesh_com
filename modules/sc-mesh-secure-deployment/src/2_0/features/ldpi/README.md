# Lightweight Deep Packet Inspection (LDPI)

Lightweight Deep Packet Inspection (LDPI) is a component within the Comm. Sleeve mesh_com repository. It focuses on
network traffic analysis at Layer 3, employing anomaly detection and machine learning techniques.

## Components

#### LightDeepPacketInspection

Acts as the principal subscriber to the network sniffer. It processes network flows and deploys a machine learning model
for anomaly detection.

#### Sniffer

A module dedicated to capturing and processing network packets. It supports various protocols and manages both TCP and
UDP flows, including handling fragmented IP packets.

#### TrainedModel

Manages the machine learning model used for anomaly detection. It is responsible for the inference process to identify
anomalous activities within the network flows.

## Training Component

The `ldpi/training` directory encompasses all the necessary code for training the anomaly detection model. The training
process involves two primary steps:

1. **Data Preparation**: Place your dataset in the `datasets/` folder, e.g., `ldpi/datasets/TII-SSRC-23/`. The
   dataset should contain two subfolders: 'benign' and 'malicious'. The model is trained exclusively on the benign data,
   while the malicious data is used for assessing the model's performance. It is highly recommended to follow the file
   structure (organization of the pcap files) the same way as in the TII-SSRC-23 dataset.

2. **Preprocessing**: Executed through `ldpi/training/preprocessing.py`. This script prepares the network data for
   training.

3. **Model Training**: Performed by running `ldpi/training/training.py`. This script takes the preprocessed data and
   trains the model.

The current pretrained model in the repository was trained on the [TII-SSRC-23 dataset](https://www.kaggle.com/datasets/daniaherzalla/tii-ssrc-23). It should be tested with those .pcap files. If new traffic is targeted, a significant amount of traffic must be gathered for retraining the model.

## Testing

To run the tests, it may be necessary to set the Python export path (EXPORTPATH) to ensure proper functioning of imports.

## Modes

- **Traffic From Interface (`main.py`)**: Operates as the expected LDPI, sniffing the network interface. It will be instantiated by the decision engine.
- **Traffic From `.pcap`  (`main_debug.py`)**: A tool for testing the approach in which the sniff function of the sniffer is overwritten to read packets from .pcap files. This allows for easy debugging and reproducibility of the real-time approach with source traffic coming from gathered traffic.
