from mutauth import *
from tools.monitoring_wpa import WPAMonitor
import argparse
import subprocess

shutdown_event = threading.Event()
cbma_threads = []
file_dir = os.path.dirname(__file__) # Path to dir containing this script

def setup_macsec(level, interface_name, port, batman_interface, path_to_certificate, path_to_ca, macsec_encryption, wpa_supplicant_control_path = None):
    '''
    Sets up macsec links between peers for an interface and adds the macsec interfaces to batman_interface
    '''

    wait_for_interface_to_be_up(interface_name)  # Wait for interface to be up, if not already
    in_queue = queue.Queue()  # Queue to store wpa peer connected messages/ multicast messages on interface for cbma
    mua = mutAuth(in_queue, level=level, meshiface=interface_name, port=port,
                  batman_interface=batman_interface, path_to_certificate=path_to_certificate, path_to_ca=path_to_ca, macsec_encryption=macsec_encryption, shutdown_event=shutdown_event)
    # Wait for wireless interface to be pingable before starting mtls server, multicast
    wait_for_interface_to_be_pingable(mua.meshiface, mua.ipAddress)
    # Start server to facilitate client auth requests, monitor ongoing auths and start client request if there is a new peer/ server baecon
    auth_server_thread, auth_server = mua.start_auth_server()

    if wpa_supplicant_control_path:
        # Start monitoring wpa for new peer connection
        wpa_ctrl_instance = WPAMonitor(wpa_supplicant_control_path)
        wpa_thread = threading.Thread(target=wpa_ctrl_instance.start_monitoring, args=(in_queue, shutdown_event))
        wpa_thread.start()

    # Start multicast receiver that listens to multicasts from other mesh nodes
    receiver_thread = threading.Thread(target=mua.multicast_handler.receive_multicast)
    monitor_thread = threading.Thread(target=mua.monitor_wpa_multicast)
    receiver_thread.start()
    monitor_thread.start()
    # Send periodic multicasts
    mua.sender_thread.start()
    return mua

def cbma(level, interface_name, port, batman_interface, path_to_certificate, path_to_ca, macsec_encryption, wpa_supplicant_control_path=None):
    '''
        Sets up macsec and batman for the specified interface and level
        level: MACSec/ CBMA level. "lower" or "upper"
        interface_name: Name of the interface (physical interface if lower level, bat0 if upper level)
        port: Port number for mutual authentication and multicast. Can use 15001 for lower level and 15002 for upper level
        batman_interface: Batman interface name. bat0 for lower level, bat1 for upper level
        path_to_certificate: Path to folder containing certificates
        path_to_ca: Path to ca certificate
        macsec_encryption: Encryption flag for macsec. "on" or "off"
        wpa_supplicant_control_path: Path to wpa supplicant control (if any)
        '''
    mutauth_obj = setup_macsec(level, interface_name, port, batman_interface, path_to_certificate, path_to_ca, macsec_encryption, wpa_supplicant_control_path)
    mutauth_obj.setup_batman()
    return mutauth_obj

def main():
    '''
    Example of setting up cbma for interfaces wlp1s0, eth1

    Prerequisite: test certificates generation
        1. Connect CSls/ CMs to your PC with ethernet
        2. Run cbma/cert_generation/ca_side.py in your PC
        3. Run cbma/cert_generation/client_side.py --interface {interface} in your CSLs/ CMs for each interface that you want to apply cbma to
    '''
    # Change mode of scripts to executable (Needs to be done once initially)
    subprocess.run(['chmod', '+x', f'{file_dir}/delete_default_brlan_bat0.sh'])
    subprocess.run(['chmod', '+x', f'{file_dir}/cleanup_cbma.sh'])

    # Delete default bat0 and br-lan if they come up by default
    subprocess.run([f'{file_dir}/delete_default_brlan_bat0.sh'])

    # Apply firewall rules that only allows macsec traffic and cbma configuration traffic
    # TODO: nft needs to be enabled on the images
    #apply_nft_rules(rules_file=f'{file_dir}/tools/firewall.nft')

    # Start cbma lower for each interface/ radio by calling cbma(), which in turn calls setup_macsec(), followed by setup_batman()
    # For example, for wlp1s0:
    cbma_wlp1s0 = threading.Thread(
        target=cbma,
        args=(
            "lower", # level
            "wlp1s0", # interface_name
            15001, # port
            "bat0", # batman_interface
            f'{file_dir}/cert_generation/certificates', # path_to_certificate
            f'{file_dir}/cert_generation/certificates/ca.crt', # path_to_ca
            "off", # macsec_encryption (can be "on" if required)
            '/var/run/wpa_supplicant_id0/wlp1s0' # wpa_supplicant_control_path
        ),
    )
    cbma_threads.append(cbma_wlp1s0)
    cbma_wlp1s0.start()

    # Similarly, for eth1
    cbma_eth1 = threading.Thread(
        target=cbma,
        args=(
            "lower",
            "eth1",
            15001,
            "bat0",
            f'{file_dir}/cert_generation/certificates',
            f'{file_dir}/cert_generation/certificates/ca.crt',
            "off",
            None
        ),
    )
    #cbma_eth1 = threading.Thread(target=cbma, args=("lower", "eth1", 15001, "bat0", f'{file_dir}/cert_generation/certificates', f'{file_dir}/cert_generation/certificates/ca.crt', "off", None))
    cbma_threads.append(cbma_eth1)
    cbma_eth1.start()
    # Repeat the same for other interfaces/ radios by changing the interface_name and wpa_supplicant_control_path (if any). The port number can be reused for the lower level
    # setup_batman will setup bat0 once (for whichever interface this is called first) by setting the mac address of bat0 same as that of the physical interface
    # This is because right now, we reuse certificates from one of the physical interfaces for upper macsec over bat0
    # setup_batman may be modified later if required

    # Start cbma upper for bat0 by calling cbma() for lower batman interface (bat0)
    # This only needs to be called once
    # path_to_certificates and path_to_ca can be changed as required
    # batman interface = bat1, port number should be different from that used for lower cbma
    cbma_bat0 = threading.Thread(
        target=cbma,
        args=(
            "upper",
            "bat0",
            15002,
            "bat1",
            f'{file_dir}/cert_generation/certificates',
            f'{file_dir}/cert_generation/certificates/ca.crt',
            "on",
            None
        ),
    )
    cbma_threads.append(cbma_bat0)
    cbma_bat0.start()

def stop():
    shutdown_event.set()
    for cbma_thread in cbma_threads:
        cbma_thread.join()
    # Delete macsec links, batman interfaces and bridges created within CBMA
    subprocess.run([f'{file_dir}/cleanup_cbma.sh'])

if __name__ == "__main__":
    main()