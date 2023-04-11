import sys
import json
from csa import Csa_Server
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
    if len(sys.argv) < 2:
        print("Usage: python csa-server.py <interface name>")
        sys.exit(1)
    ifname = sys.argv[1]

    myipaddr = get_ipv6_address(ifname)
    server = Csa_Server(myipaddr, 8000)
    seq = 1

    while True:
        try:
            conn = server.accept()

            while True:
                data = conn.recv(1024)
                if not data: break
                msg = data.decode()
                print('Received CSA: ' + msg)
                # Write the channel to file
                msg_dict = json.loads(msg)
                ch = msg_dict.get('ch')
                if ch is not None:
                    with open("ch.txt", "w") as f:
                        f.write(str(ch))  
                # Send an ACK with updated seq
                msg_dict = json.loads(msg)
                response_dict = {
                    'type': 'CSA-ACK',
                    'src': myipaddr, 
                    'seq': seq
                }
                response_json = json.dumps(response_dict)
                print('Sending CSA-ACK:' + response_json)
                conn.send(response_json.encode())
                seq += 1

            conn.close()
        except ConnectionResetError as e:
            print("Error: " + str(e))
            conn.close()
            continue
