import asyncio
import nats
import json
import base64
import config


async def main():
    # Connect to NATS!
    nc = await nats.connect(f"{config.MODULE_IP}:{config.MODULE_PORT}")

    # DMESG and WPA currently supported
    rep = await nc.request("comms.command",
                            b"""{"api_version": 1,"cmd": "LOGS", "param": "DMESG"}""",
                            timeout=2)

    print(rep.data)
    parameters = json.loads(rep.data.decode())
    b64_data = base64.b64decode(parameters["data"].encode())
    print(b64_data.decode())

    rep = await nc.request("comms.command",
                            b"""{"api_version": 1,"cmd": "LOGS", "param": "WPA"}""",
                            timeout=2)
    print(rep.data)
    parameters = json.loads(rep.data.decode())
    b64_data = base64.b64decode(parameters["data"].encode())
    print(b64_data.decode())

    await nc.close()
    exit(0)

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.run_forever()
    loop.close()

