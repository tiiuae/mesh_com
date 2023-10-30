import asyncio
import client
import json
import config


async def main():
    # Connect to NATS!
    nc = await client.connect_nats()

    cmd_dict = {"api_version": 1, "cmd": "REVOKE"}
    cmd = json.dumps(cmd_dict)
    rep = await nc.request(f"comms.command.{config.MODULE_IDENTITY}",
                           cmd.encode(),
                           timeout=4)
    parameters = json.loads(rep.data)
    print(parameters)

    await nc.close()
    exit(0)

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.close()
