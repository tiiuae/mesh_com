"""
Comms NATS controller for FMO
"""
import argparse
import asyncio
import json
import signal
import ssl

from nats.aio.client import Client as nats

from src import comms_controller

# FMO_SUPPORTED_COMMANDS = [
#     "GET_IDENTITY",
#     "ENABLE_VISUALISATION",
#     "DISABLE_VISUALISATION",
# ]


# pylint: disable=too-many-arguments, too-many-locals, too-many-statements
async def main_fmo(server, port, keyfile=None, certfile=None, interval=1000) -> None:
    """
    main fmo
    param: server: server
    param: port: port
    param: keyfile: keyfile
    param: certfile: certfile
    param: interval: interval
    return: -
    """
    cc = comms_controller.CommsController(interval)

    nats_client = nats()
    status, _, identity_dict = cc.command.get_identity()

    if status == "OK":
        identity = identity_dict["identity"]
        cc.logger.debug("Identity: %s", identity)
    else:
        cc.logger.error("Failed to get identity!")
        return

    async def stop():
        await asyncio.sleep(1)
        asyncio.get_running_loop().stop()

    def signal_handler():
        if nats_client.is_closed:
            return
        cc.logger.debug("Disconnecting...")
        asyncio.create_task(nats_client.close())
        asyncio.create_task(stop())

    for sig in ("SIGINT", "SIGTERM"):
        asyncio.get_running_loop().add_signal_handler(
            getattr(signal, sig), signal_handler
        )

    async def disconnected_cb():
        cc.logger.debug("Got disconnected...")

    async def reconnected_cb():
        cc.logger.debug("Got reconnected...")

    # Create SSL context if certfile and keyfile are provided
    ssl_context = None
    if certfile and keyfile:
        ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ssl_context.load_cert_chain(certfile=certfile, keyfile=keyfile)

    # Connect to NATS server with TLS enabled if ssl_context is provided
    if ssl_context:
        await nats_client.connect(
            f"tls://{server}:{port}",
            tls=ssl_context,
            reconnected_cb=reconnected_cb,
            disconnected_cb=disconnected_cb,
            max_reconnect_attempts=-1,
        )
    else:
        await nats_client.connect(
            f"nats://{server}:{port}",
            reconnected_cb=reconnected_cb,
            disconnected_cb=disconnected_cb,
            max_reconnect_attempts=-1,
        )

    async def fmo_message_handler(message):
        """
        Message handler for FMO
        """
        # reply = message.reply
        subject = message.subject
        data = message.data.decode()
        cc.logger.debug("Received a message on '%s': %s", subject, data)
        ret, info, resp = "FAIL", "Not supported subject", ""

        if subject == f"comms.settings.{identity}":
            ret, info = cc.settings.handle_mesh_settings(data)
        elif subject == f"comms.channel_change.{identity}":
            ret, info, _index = cc.settings.handle_mesh_settings_channel_change(data)
            if ret == "TRIGGER":
                cmd = json.dumps(
                    {"api_version": 1, "cmd": "APPLY", "radio_index": f"{_index}"}
                )
                ret, info, resp = cc.command.handle_command(cmd, cc)
        elif subject in (f"comms.command.{identity}", "comms.identity"):
            # this file was saved by mdm
            try:
                with open(
                    "/opt/debug_config.json", "r", encoding="utf-8"
                ) as debug_conf_json:
                    debug_conf_data = debug_conf_json.read().strip()
                    debug_conf_data_json = json.loads(debug_conf_data)
                    debug_mode = debug_conf_data_json["payload"]["debug_conf"][0][
                        "debug_mode"
                    ]
                    cc.debug_mode_enabled = True if debug_mode == "enabled" else False
            except Exception as e:
                cc.logger.error("Error reading debug config file: %s", e)
            ret, info, resp = cc.command.handle_command(data, cc)
        elif subject == f"comms.status.{identity}":
            ret, info = "OK", "Returning current status"

        # Update status info
        _ = [item.refresh_status() for item in cc.c_status]

        response = {
            "status": ret,
            "info": info,
            "mesh_status": [item.mesh_status for item in cc.c_status],
            "mesh_cfg_status": [item.mesh_cfg_status for item in cc.c_status],
            "visualisation_active": [
                item.is_visualisation_active for item in cc.c_status
            ],
            "mesh_radio_on": [item.is_mesh_radio_on for item in cc.c_status],
            "ap_radio_on": [item.is_ap_radio_on for item in cc.c_status],
            "security_status": [item.security_status for item in cc.c_status],
        }

        if resp != "":
            response["data"] = resp

        cc.logger.debug("Sending response: %s", str(response)[:1000])
        await message.respond(json.dumps(response).encode("utf-8"))

    await nats_client.subscribe(f"comms.settings.{identity}", cb=fmo_message_handler)
    await nats_client.subscribe(
        f"comms.channel_change.{identity}", cb=fmo_message_handler
    )
    await nats_client.subscribe(f"comms.status.{identity}", cb=fmo_message_handler)
    await nats_client.subscribe("comms.identity", cb=fmo_message_handler)
    await nats_client.subscribe(f"comms.command.{identity}", cb=fmo_message_handler)

    cc.logger.debug("FMO comms_nats_controller Listening for requests")

    while True:
        await asyncio.sleep(float(cc.interval) / 1000.0)
        try:
            if cc.telemetry.visualisation_enabled:
                msg = cc.telemetry.mesh_visual()
                cc.logger.debug(f"Publishing comms.visual.{identity}: %s", msg)
                await nats_client.publish(f"comms.visual.{identity}", msg.encode())
        except Exception as e:
            cc.logger.error("Error:", e)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Mesh Settings")
    parser.add_argument("-s", "--server", help="Server IP", required=False)
    parser.add_argument("-p", "--port", help="Server port", required=False)
    parser.add_argument("-k", "--keyfile", help="TLS keyfile", required=False)
    parser.add_argument("-c", "--certfile", help="TLS certfile", required=False)
    parser.add_argument("-i", "--interface", help="e.g. br-lan or bat0", required=False)

    args = parser.parse_args()
    loop = asyncio.new_event_loop()

    loop.run_until_complete(
        main_fmo(args.server, args.port, args.keyfile, args.certfile)
    )
    loop.close()
