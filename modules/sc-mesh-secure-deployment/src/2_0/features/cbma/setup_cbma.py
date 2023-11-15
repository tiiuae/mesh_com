from mutauth import *
from tools.monitoring_wpa import WPAMonitor
import argparse

shutdown_event = threading.Event()

file_dir = os.path.dirname(__file__) # Path to dir containing this script

def cbma(level, interface_name, port, batman_interface, path_to_certificate, wpa_supplicant_control_path = None):
    '''
    level: MACSec/ CBMA level. lower or upper
    interface_name: Name of the interface
    port: Port number for mutual authentication and multicast. Can use 15001 for lower level and 15002 for upper level
    batman_interface: Batman interface name. bat0 for lower level, bat1 for upper level
    path_to_certificate: Path to folder containing certificates
    wpa_supplicant_control_path: Path to wpa supplicant control (if any)
    '''

    wait_for_interface_to_be_up(interface_name)  # Wait for interface to be up, if not already
    in_queue = queue.Queue()  # Queue to store wpa peer connected messages/ multicast messages on interface for cbma
    mua = mutAuth(in_queue, level=level, meshiface=interface_name, port=port,
                  batman_interface=batman_interface, path_to_certificate=path_to_certificate, shutdown_event=shutdown_event)
    # Wait for wireless interface to be pingable before starting mtls server, multicast
    wait_for_interface_to_be_pingable(mua.meshiface, mua.ipAddress)
    # Start server to facilitate client auth requests, monitor ongoing auths and start client request if there is a new peer/ server baecon
    auth_server_thread, auth_server = mua.start_auth_server()

    if wpa_supplicant_control_path:
        # Start monitoring wpa for new peer connection
        wpa_ctrl_instance = WPAMonitor(wpa_supplicant_control_path)
        wpa_thread = threading.Thread(target=wpa_ctrl_instance.start_monitoring, args=(in_queue,))
        wpa_thread.start()

    # Start multicast receiver that listens to multicasts from other mesh nodes
    receiver_thread = threading.Thread(target=mua.multicast_handler.receive_multicast)
    monitor_thread = threading.Thread(target=mua.monitor_wpa_multicast)
    receiver_thread.start()
    monitor_thread.start()
    # Send periodic multicasts
    mua.sender_thread.start()



def main():
    # Delete default bat0 and br-lan if they come up by default
    subprocess.run([f'{file_dir}/delete_default_brlan_bat0.sh'])

    # Apply firewall rules that only allows macsec traffic and cbma configuration traffic
    # TODO: nft needs to be enabled on the images
    #apply_nft_rules(rules_file=f'{file_dir}/tools/firewall.nft')

    # Start cbma lower for each interface/ radio
    # For example, for wlp1s0:
    cbma(level="lower", interface_name="wlp1s0", port=15001, batman_interface="bat0", path_to_certificate=f'{file_dir}/cert_generation/certificates', wpa_supplicant_control_path='/var/run/wpa_supplicant_id0/wlp1s0')
    # Repeat the same by changing the interface_name and wpa_supplicant_control_path (if any) for other interfaces/ radios. The port number can be reused for the lower level


    # Start cbma upper for bat0
    # This only needs to be called once
    cbma(level="upper", interface_name="bat0", port=15002, batman_interface="bat1", path_to_certificate=f'{file_dir}/cert_generation/certificates')

if __name__ == "__main__":
    main()