import asyncio
import nats
import json
import config


async def main():
    # Connect to NATS!
    nc = await nats.connect(f"{config.MODULE_IP}:{config.MODULE_PORT}")
    cmd_dict = {"api_version": 1, "cmd": "DISABLE_VISUALISATION", "interval": "1000"}
    cmd = json.dumps(cmd_dict)
    rep = await nc.request(f"comms.command.{config.MODULE_IDENTITY}",
                           cmd.encode(),
                           timeout=2)
    parameters = json.loads(rep.data)
    print(parameters)
    await nc.close()
    exit(0)


if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.run_forever()
    loop.close()
