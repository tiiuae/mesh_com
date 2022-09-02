#!/usr/bin/python
import contextlib
from . import server
from . import client


class CA:

    def __init__(self, ID):
        self.ID = ID

    def as_client(self, server_ip, return_dict):
        client.initiate_client(server_ip, self.ID, return_dict)

    def as_server(self, ip):
        with contextlib.suppress(OSError):
            server.initiate_server(ip)