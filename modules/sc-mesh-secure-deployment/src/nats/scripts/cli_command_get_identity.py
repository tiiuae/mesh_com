import asyncio
import nats
import json
import base64
import config


async def main():
    # Connect to NATS!
    nc = await nats.connect(f"{config.MODULE_IP}:{config.MODULE_PORT}")

    cmd_dict = {"api_version": 1, "cmd": "GET_IDENTITY"}
    cmd = json.dumps(cmd_dict)
    rep = await nc.request("comms.command",
                            cmd.encode(),
                            timeout=2)
    print(rep.data)

    parameters = json.loads(rep.data.decode())
    print(json.dumps(parameters, indent=2))

    await nc.close()
    exit(0)

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.run_forever()
    loop.close()


