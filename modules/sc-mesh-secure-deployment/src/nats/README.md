# NATS Clients

## Overview

This folder contains the NATS clients for the Secure Deployment module.

## Requirements
If executing UnitTests in a device, please stop the MDM agent:
```
/opt/S90mdm_agent stop
```
## Unit Tests

### Option 1 (PC development environment)
On your PC:
```
./run_unittests_PC.sh
```

### Option 2 (in target device)
To execute unit tests, run the following command from the root of the nats folder:
```
python3 -m unittest discover -v
```

### Option 3 (pytest)
or with pytest-3 (pretty output): 
```
sudo apt install python3-pytest
pytest-3 ./tests/
```

### Option 4 (single tests)
or single case example:
```
python3 -m unittest tests.test_validation.TestValidation.test_validate_tx_ssid -v
```
### *HOX!  Some of the tests can be executed only as root*