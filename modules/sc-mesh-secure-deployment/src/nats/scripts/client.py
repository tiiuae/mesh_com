import ssl
import os
import nats
from nats.aio.client import Client
import config

client_cert = "/etc/ssl/certs/comms_auth_cert.pem"
key = "/etc/ssl/private/comms_auth_private_key.pem"
ca_cert = "/etc/ssl/certs/root-ca.cert.pem"


async def connect_nats():
    if os.path.exists(client_cert) and \
       os.path.exists(key) and \
       os.path.exists(ca_cert):

        ssl_context = ssl.create_default_context()
        ssl_context.load_cert_chain(certfile=client_cert, keyfile=key)
        ssl_context.load_verify_locations(cafile=ca_cert)

        nats_client = await nats.connect(f"{config.MODULE_IP}:{config.MODULE_PORT}",
                                         tls=ssl_context)

    else:
        nats_client = await nats.connect(f"{config.MODULE_IP}:{config.MODULE_PORT}")

    return nats_client


async def connect(nats_client: Client, recon_cb=None, closed_cb=None, max_recon_attempts=None):
    if os.path.exists(client_cert) and \
       os.path.exists(key) and \
       os.path.exists(ca_cert):

        ssl_context = ssl.create_default_context()
        ssl_context.load_cert_chain(certfile=client_cert, keyfile=key)
        ssl_context.load_verify_locations(cafile=ca_cert)

        await nats_client.connect(f"{config.MODULE_IP}:{config.MODULE_PORT}",
                                  tls=ssl_context,
                                  reconnected_cb=recon_cb,
                                  closed_cb=closed_cb,
                                  max_reconnect_attempts=max_recon_attempts)

    else:
        await nats_client.connect(f"{config.MODULE_IP}:{config.MODULE_PORT}",
                                  reconnected_cb=recon_cb,
                                  closed_cb=closed_cb,
                                  max_reconnect_attempts=max_recon_attempts)

    return None
