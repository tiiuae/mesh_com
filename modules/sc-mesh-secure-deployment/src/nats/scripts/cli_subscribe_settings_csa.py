import asyncio
import signal
import json
from nats.aio.client import Client as NATS
import config

async def run(loop):
    nc = NATS()

    async def error_cb(e):
        print("Error:", e)

    async def closed_cb():
        print("Connection to NATS is closed.")
        if nc.is_closed:
            return
        await nc.close()

    async def reconnected_cb():
        print(f"Connected to NATS ...")

    async def subscribe_handler(msg):
        subject = msg.subject
        reply = msg.reply
        data = json.loads(msg.data.decode())
        print("Received a message on '{subject} {reply}': {data}".format(
          subject=subject, reply=reply, data=data))

    try:
        await nc.connect(f"nats://{config.MODULE_IP}:{config.MODULE_PORT}",
                         reconnected_cb=reconnected_cb,
                         closed_cb=closed_cb,
                         max_reconnect_attempts=-1)
    except Exception as e:
        print(e)

    print(f"Connected to NATS at: {nc.connected_url.netloc}")

    def signal_handler():
        if nc.is_closed:
            return
        print("Disconnecting...")
        loop.create_task(nc.close())

    for sig in ('SIGINT', 'SIGTERM'):
        loop.add_signal_handler(getattr(signal, sig), signal_handler)

    await nc.subscribe("comms.settings_csa.>", "", cb=subscribe_handler)

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(loop))
    try:
        loop.run_forever()
    finally:
        loop.close()