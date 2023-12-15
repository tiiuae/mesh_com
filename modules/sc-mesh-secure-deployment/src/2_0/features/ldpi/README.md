# Lightweight Deep Packet Inspection (LDPI)

## Introduction

Lightweight Deep Packet Inspection (LDPI) is a component within the Comm. Sleeve mesh_com repository. It focuses on
network traffic analysis through deep packet inspection methodologies, employing machine learning techniques for
enhanced security measures.

## Components

### LightDeepPacketInspection

Acts as the principal subscriber to the network sniffer. It processes network flows and deploys a machine learning model
for anomaly detection.

### Sniffer

A module dedicated to capturing and processing network packets. It supports various protocols and manages both TCP and
UDP flows, including handling fragmented IP packets.

### TrainedModel

Manages the machine learning model used for anomaly detection. It is responsible for the inference process to identify
anomalous activities within the network flows.

## Training Component

The `ldpi/training` directory encompasses all the necessary code for training the anomaly detection model. The training
process involves two primary steps:

1. **Data Preparation**: Place your dataset in the `datasets/` folder, e.g., `ids-ldpi/datasets/TII-SSRC-23/`. The
   dataset should contain two subfolders: 'benign' and 'malicious'. The model is trained exclusively on the benign data,
   while the malicious data is used for assessing the model's performance. It is highly recommended to follow the file
   structure (organization of the pcap files) the same way as in the TII-SSRC-23 dataset.

2. **Preprocessing**: Executed through `ldpi/training/preprocessing.py`. This script prepares the network data for
   training.

3. **Model Training**: Performed by running `ldpi/training/training.py`. This script takes the preprocessed data and
   trains the model.
