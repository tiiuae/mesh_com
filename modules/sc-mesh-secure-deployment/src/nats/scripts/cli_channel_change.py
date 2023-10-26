import asyncio
import client
import json
import config

async def main():
    # Connect to NATS!
    nc = await client.connect_nats()
    cmd_dict = {"frequency": "2412", "radio_index": "0"}
    cmd = json.dumps(cmd_dict)
    rep = await nc.request(f"comms.channel_change.{config.MODULE_IDENTITY}", cmd.encode(), timeout=10)
    print(f"Published to comms.channel_change: {cmd}")
    parameters = json.loads(rep.data)
    print(json.dumps(parameters, indent=2))
    await nc.close()
    exit(0)


if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.close()
