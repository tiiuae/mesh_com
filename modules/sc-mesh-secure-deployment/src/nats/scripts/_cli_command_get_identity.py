import asyncio
import nats
import json
import config

async def main():
    # Connect to NATS!
    nc = await nats.connect(f"{config.MODULE_IP}:{config.MODULE_PORT}")

    cmd_dict = {"api_version": 1, "cmd": "GET_IDENTITY"}
    cmd = json.dumps(cmd_dict)
    rep = await nc.request("comms.command.>",
                            cmd.encode(),
                            timeout=2)
    print(rep.data)

    parameters = json.loads(rep.data.decode())
    print(json.dumps(parameters, indent=2))

    if "identity" in parameters["data"]:
        with open("identity.py", "w") as f:
            f.write(f"MODULE_IDENTITY=\"{parameters['data']['identity']}\"\n")
    else:
        print("No identity received!!!!!!!!!!!")
    await nc.close()
    exit(0)

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.run_forever()
    loop.close()
