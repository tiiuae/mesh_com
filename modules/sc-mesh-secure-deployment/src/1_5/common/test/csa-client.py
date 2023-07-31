import sys
import json
from csa import Csa_Client
import netifaces

def get_ipv6_address(interface_name):
    # Get a dictionary containing all the addresses associated with the interface
    addresses = netifaces.ifaddresses(interface_name)

    # Get the IPv6 address associated with the interface, if it exists
    if netifaces.AF_INET6 in addresses:
        for address in addresses[netifaces.AF_INET6]:
            if 'addr' in address:
                return address['addr']

    # Return None if no IPv6 address was found
    return None

if __name__ == '__main__':
    # Get the IPv6 address from the command line arguments
    if len(sys.argv) < 4:
        print("Usage: python csa-client.py <interface name> <dest address> <new channel>")
        sys.exit(1)
    ifname = sys.argv[1]
    ipdest = sys.argv[2]
    ch = sys.argv[3]

    myipaddr = get_ipv6_address(ifname)
    client = Csa_Client(ipdest, 8000)
    client.connect()

    msg = {
        'type': 'CSA',
        'src': myipaddr,
        'ch': ch
    }
    
    client.send(json.dumps(msg).encode())
    
    while True:
        data = client.recv()
        if not data:
            break
        print("Received ACK: " + data.decode())
        client.close()
        sys.exit()
