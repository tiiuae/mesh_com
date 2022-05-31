#!/usr/bin/python
import server
import client


class CA:

    def as_client(self,server_ip):
        client.initiate_client(server_ip)

    def as_server(self,ip):
        server.initiate_server(ip)
