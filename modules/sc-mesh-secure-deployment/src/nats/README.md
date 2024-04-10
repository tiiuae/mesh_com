# NATS Clients

## Overview

This folder contains the NATS clients for the Secure Deployment module.

## Unit Tests

To execute unit tests, run the following command from the root of the nats folder:

```
python3 -m unittest discover
```
or with pytest-3 (pretty output): 
```
sudo apt install python3-pytest
pytest-3 ./tests/
```
or single case example:
```
python3 -m unittest tests.test_validation.TestValidation.test_validate_tx_ssid -v
```
### *HOX!  Some of the tests can be executed only as root*