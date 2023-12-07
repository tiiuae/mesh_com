import asyncio
import client
import json
import base64
import config


async def main():
    # Connect to NATS!
    nc = await client.connect_nats()

    cmd_dict = {"api_version": 1, "cmd": "LOGS", "param": "WPA", "radio_index": "0"}
    cmd = json.dumps(cmd_dict)
    rep = await nc.request(f"comms.command.{config.MODULE_IDENTITY}",
                           cmd.encode(),
                           timeout=2)
    print(rep.data)
    parameters = json.loads(rep.data.decode())
    if parameters["data"] is None:
        print("No logs available")
    else:
        b64_data = base64.b64decode(parameters["data"].encode())
        print(b64_data.decode())

    await nc.close()
    exit(0)

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.close()
