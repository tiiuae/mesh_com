# For a Functional testing

## Overview

This folder includes initial functional tests.  Test is utilizing NATS.io
interface to send messages to Comms Controller.  

## Functional Tests

### To execute

- Please set correct device lan1/eth0 IP address to config.py
- then execute
```
python3 -m unittest discover --verbose
```

## Coverage

    async def test_identity(self):
    async def test_status(self):
    async def test_settings_and_apply(self):
    async def test_get_configs(self):
    async def test_visual(self):
    async def test_logs(self):
    async def test_channel_change(self):

if __name__ == "__main__":
    unittest.main()

