import asyncio
import json
import ssl
import concurrent.futures
from nats.aio.client import Client as NATS

class ridNatsClient:
    def __init__(self, server, port, certfile=None, keyfile=None):
        self.server = server
        self.port = port
        self.certfile = certfile
        self.keyfile = keyfile
        self.nc = NATS()
        self.loop = asyncio.get_event_loop()

    async def connect(self):
        # Create SSL context if certfile and keyfile are provided
        ssl_context = None
        if self.certfile and self.keyfile:
            ssl_context = ssl.create_default_context()
            ssl_context.load_cert_chain(certfile=self.certfile, keyfile=self.keyfile)

        # Connect to NATS server with TLS enabled if ssl_context is provided
        if ssl_context:
            await self.nc.connect(f"tls://"+str(self.server)+":"+str(self.port), loop=self.loop, ssl=ssl_context)
        else:
            await self.nc.connect(f"nats://"+str(self.server)+":"+str(self.port))

    async def publish_async(self, topic, data):
        # Convert data to JSON string
        json_payload = json.dumps(data)

        # Publish message
        await self.nc.publish(topic, bytes(json_payload.encode('utf-8')))
        #print(json_payload)

    def publish(self, topic, data):
        # Use a thread pool to run publish_async in a separate thread
        with concurrent.futures.ThreadPoolExecutor() as executor:
            loop = asyncio.get_event_loop()
            future = loop.run_in_executor(executor, self.publish_async, topic, data)
            return future.result()

    async def close(self):
        # Close connection to NATS server
        await self.nc.close()

def main():
    # Create NATS client instance
    # Add cert/key file later for TLS support
    client = ridNatsClient(server="127.0.0.1", port=4443, certfile="", keyfile="")

    # Connect to NATS server
    client.loop.run_until_complete(client.connect())

    # Publish rid message to a NATS topic
    data = {
        "timestamp": self.timestamp,
        "operational_status": self.operational_status,
        "position": {
            "lat": self.position_lat,
            "lng": self.position_lng,
            "alt": self.position_alt,
            "accuracy_h": self.position_accuracy_h,
            "accuracy_v": self.position_accuracy_v,
            "extrapolated": self.position_extrapolated
        },
        "height": {
            "distance": self.height_distance,
            "reference": self.height_reference
        },
        "track": self.track,
        "speed": self.speed,
        "timestamp_accuracy": self.timestamp_accuracy,
        "speed_accuracy": self.speed_accuracy,
        "vertical_speed": self.vertical_speed
    }

    client.loop.run_until_complete(client.publish("my_topic", data))

    # Close connection to NATS server
    client.loop.run_until_complete(client.close())

if __name__ == '__main__':
    main()

