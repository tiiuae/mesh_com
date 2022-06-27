#!/usr/bin/python
from . import server
from . import client


class CA:

    def __init__(self, ID):
        self.ID = ID


    def as_client(self, server_ip):
        return client.initiate_client(server_ip, self.ID)

    def as_server(self, ip):
        server.initiate_server(ip)
