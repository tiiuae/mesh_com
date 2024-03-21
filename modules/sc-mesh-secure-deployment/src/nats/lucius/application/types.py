from typing import Callable

BATMAN_INTERFACES = ["bat0", "bat1"]

NODE_DATA_REQUEST = "GET"

SocketSendType = Callable[[str], None]

ResponseCallback = Callable[[str, SocketSendType], None]