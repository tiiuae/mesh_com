import asyncio
import nats
import json
import config

async def main():
    # Connect to NATS!
    nc = await nats.connect(f"{config.MODULE_IP}:{config.MODULE_PORT}")
    rep = await nc.request("comms.command",
                            b"""{"api_version": 1,"cmd": "ENABLE_VISUALISATION", "interval": "1000"}""",
                            timeout=4)
    parameters = json.loads(rep.data.decode())
    print(parameters)
    await nc.close()
    exit(0)

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.run_forever()
    loop.close()