import asyncio
import nats
import json
import config


async def main():
    # Connect to NATS!
    nc = await nats.connect(f"{config.MODULE_IP}:{config.MODULE_PORT}")
    cmd_dict = {"frequency": "2472", "delay": "1", "amount": "2"}
    cmd = json.dumps(cmd_dict)
    rep = await nc.publish("comms.settings_csa", cmd.encode())
    print(f"Published to comms.settings_csa: {cmd} ({rep})")
    await nc.close()
    exit(0)


if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.run_forever()
    loop.close()
