#!/usr/bin/python
import contextlib
from . import server
from . import client


class CA:

    def __init__(self, ID):
        self.ID = ID

    def as_client(self, server_ip):
        client.initiate_client(server_ip, self.ID)

    def as_server(self, ip, return_dict):
        with contextlib.suppress(OSError):
            server.initiate_server(ip, return_dict)
            