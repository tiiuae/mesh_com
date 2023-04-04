# Jamming Detection using Deep Learning with Time Series Data from Spectral Analyzer
This is a project for jamming detection using deep learning with time series data from a spectral analyzer. The project uses PyTorch and the tsai library to train and test the model.

## Requirements
* PyTorch
* tsai
* sklearn
* matplotlib

## Usage
Install the required packages.
Run the script.
```
python main.py
```

## Model
The model used in this project is a custom Separable CNN, ResCNN, InceptionTime, and others DL models for time series. The input of the model is a time series from a spectral analyzer. The output of the model is a classification of the input signal into one of 22 classes, representing the different types of signals that can be detected.

## Data
The data used in this project consists of time series from a spectral analyzer. The time series are preprocessed to remove noise and resampled to a fixed length. The data is split into training, validation, and test sets.

## Training
The model is trained using the AdamW optimizer with a learning rate of 0.001 and a weight decay of 0.00001. The learning rate is decreased using the cosine annealing scheduler. The loss function used is the cross-entropy loss.

## Evaluation
The performance of the model is evaluated using the test set. The accuracy of the model is calculated for both the multiclass classification and the binary classification (normal vs. all).

# 