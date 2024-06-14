# For a Debug testing

## Overview

This folder includes initial debug tests.
By default it is not allowed to run DEBUG COMMAND but you can get permissions (`debug_conf`) from MDM server.
Test is utilizing NATS.io interface to send messages to Comms Controller.

## Debug Tests

### To execute

- Please set correct device lan1/eth0 IP address to config.py
- then execute
```
python3 -m unittest discover --verbose
```

## Coverage

    async def test_identity(self):
    async def test_status(self):
    async def test_debug(self): (cat build.info command)
    async def test_slaac(self): (apply slaac command + ifconfig to check results)

if __name__ == "__main__":
    unittest.main()
