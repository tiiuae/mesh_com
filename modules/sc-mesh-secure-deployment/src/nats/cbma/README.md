# CBMA MVP Rewrite

Wow your friends and family with the new CBMA!

"It works" and "It's so simple" is what your favorite boss screams in excitement as they witness the new CBMA!


# HowTo: Run on Real Devices
```sh
# Ensure that your interface is not on a bridge
ip link set wlp1s0 nomaster

# Generate an IPv6 LLA if it doesn't have one
ip link set wlp1s0 down
sysctl -w net.ipv6.conf.wlp1s0.addr_gen_mode=1
sysctl -w net.ipv6.conf.wlp1s0.addr_gen_mode=0
ip link set wlp1s0 up

# Connect wlp1s0 to the mesh if it isn't - Need a working wpa_supplicant_11s.conf
wpa_supplicant -i wlp1s0 -c wpa_supplicant_11s.conf -D nl80211 -B

# Prepare lower-batman interface (using wlp1s0 locally administered MAC)
ip link del bat0 2>/dev/null
ip link add name bat0 type batadv
ip link set bat0 address $(read a < /sys/class/net/wlp1s0/address && printf "%02x${a:2}\n" $(( 0x${a:0:2} ^ 0x2 )))
ip link set bat0 up

# Create upper-batman inteface
ip link del bat1 2>/dev/null
ip link add name bat1 type batadv

# Install Python dependencies - Recommended to create + activate a venv before
$ python3 -m pip install -r requirements.txt

# Generate certificates for the interface you want to use if you don't have them
$ bash scripts/generate_certificates.sh wlp1s0

# Run standalone CBMA - Add -c <certs_path> if your certs aren't under `./certificates`
$ python3 standalone.py -i wlp1s0          # Runs lower-CBMA by default
# NOTE: ^ standalone.py might have to be modified if your cert dir layout is not birthcert-alike
# NOTE: File logging happens under /var/log/cbma by default, change it with the LOG_DIR env var
# NOTE: Default logging verbosity level is INFO, change it with the LOG_LEVEL env var

# Run upper-CBMA
$ python3 standalone.py -i bat0 -b bat1    # Add -u if lower-CBMA wasn't established beforehand
# NOTE: if bat0 doesn't have the same MAC as any of its attached interfaces (like LA wlp1s0 one)
#       you will have to generate certificates for it as a workaround
```


# HowTo: Run Simulation
```sh
$ sh scripts/run_simulation.sh
# NOTEL Same as above applies for logging env vars
```


# Future Improvements + Ideas
- Use a permanent TCP listener for the secure socket phase instead of only having it when macsec configuration is going to be exchanged for speed, less errors, and to prevent a DoS when one malicious peer tricks a legitimate one into starting a TCP listener, as the TCP listener stops mcast processing until it success, errors or times out (5 seconds)
- Instead of having all interfaces listening on specific ports (such as 15001), have them listening in a dedicated port derived from the last octets of their MAC address AA:BB:CC:DD:01:23 -> 0x123 -> 291 and a base one, such as 20000, so it would be listening on 20291
- Use HSM instead of filebased approach for at least private key storage
- Use raw sockets over L2 instead of L3 IPv6 and TLS



# How it works
## Multicast Phase
### Server
- Listens to UDP multicasts packets in specific port and multicast group
- For lower-CBMA this multicast group is the broadcast address
- For upper-CBMA this multicast group is defined in the Root CA (or maybe some other cert)

### Client
- Sends UDP multicast packets to specific port and multicast group
- The multicast packet content only includes the MAC address of the sender
- For lower-CBMA this multicast group is the broadcast address
- For upper-CBMA this multicast group is defined in the Root CA (or maybe some other cert)

### Role Agreement
- Server or Client role is decided depending on who has the greater MAC

### Authentication State
- The server checks if the client is already authenticated or currently authenticating

### Result
- The devices discover each other

## Secure Connection Phase
### Secure Socket
 1. **Notes**
   - Uses TLS
   - Uses the interface certificate and a birth cert chain for the secure connection
   - Mutual authentication (via `ssl` socket creation using appropriate params)
   - The port number for the connection is the same one as the Multicast service above
 2. **Server**
   - Opens TCP listener
 3. **Client**
   - Connects to server



#### Certificate creation

Create certificates for wanted interfaces with generate_certificates.sh.

Split the content of certificate_chain.crt into three files. For OpenSSL, each file must be run through openssl-command line tool
to create the format OpenSSL uses, for example:
```
openssl x509 -hash -fingerprint -in ../ca/ca_root.crt > ca_root.crt
```



### Custom Verification
 - One side verifies the other using the SSL certificate that the other side used for the connection
 - Both sides do the verification
 - TODO: To detail, but already implemented in old CBMA and the function can be copied as-is


## MACsec Phase
### Data Exchange
#### Data Generation
 1. **Bytes Generation**
   - Both sides generate random bytes for their local MACsec key
   - Both sides generate random bytes for the other device's MACsec key
 2. **Unique Port Generation**
   - Both sides get an unique TCP port that's not in use in their system

#### Data Exchange
 - The exchange is performed using the secure socket described above
 - The data exchanged is the one generated before:
   - Bytes for local MACsec key ("my key")
   - Bytes for other device's MACsec key ("client key")
   - Unique TCP port number


### Interface Creation
#### Key Derivation
 1. **Setup**
   - From the Byte Generation step above, bytes generated locally
   - From the Data Exchange above, bytes generated by the other device
 2. **Local MACsec Key**
   - Obtained by XORing:
     - The bytes of the "my key"
     - The bytes obtained from other device's "client key"
 3. **Other Device MACsec Key**
   - Obtained by XORing:
     - The bytes of "client key"
     - The bytes obtained from other device's "my key"

#### Interface Creation
 1. **MAC**
   - Uses the other device's MAC
 2. **RX**
   - Uses the unique port generated in the other device, obtained in the prior Data Exchange step
   - Uses the Other Device MACsec Key as described above
   - Key ID, why stripped client MAC and not 02?
 3. **TX**
   - Uses the unique port generated locally
   - Uses the Local MACsec Key as described above
   - Key ID: 01

### Result
- One MACSec interface is created in both devices with RX and TX keys securely exchanged
- Workaround: An additional MACSec interface is created for the broadcast traffic


## Batman Bridging Phase
### Steps
- Workaround: A VLAN interface is created between the original interface (such as wlp1s0 for lower-CBMA and bat0 for upper) and half-configured
- A bridge is created between the MACsec interface (+ the broadcast-workaround one) and the batman one
- The VLAN interface remaining configuration is performed

### Result
- The MACsec interface is added to the BATMAN interface
