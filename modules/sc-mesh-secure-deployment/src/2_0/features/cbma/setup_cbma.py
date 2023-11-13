from mutauth import *
from tools.monitoring_wpa import WPAMonitor
import argparse

shutdown_event = threading.Event()

file_dir = os.path.dirname(__file__) # Path to dir containing this script

def cbma_lower(interface_name, path_to_certificate, wpa_supplicant_control_path = None):
    in_queue_lower = queue.Queue()  # Queue to store wpa peer connected messages/ multicast messages on interface for lower cbma
    lower_batman_setup_event = threading.Event()  # Event that notifies that lower macsec and bat0 has been setup

    mua = mutAuth(in_queue_lower, level="lower", meshiface=interface_name, port=15001,
                        batman_interface="bat0", path_to_certificate=path_to_certificate, shutdown_event=shutdown_event,
                        batman_setup_event=lower_batman_setup_event)
    # Wait for wireless interface to be pingable before starting mtls server, multicast
    wait_for_interface_to_be_pingable(mua.meshiface, mua.ipAddress)
    # Start server to facilitate client auth requests, monitor ongoing auths and start client request if there is a new peer/ server baecon
    auth_server_thread, auth_server = mua.start_auth_server()

    if wpa_supplicant_control_path:
        # Start monitoring wpa for new peer connection
        wpa_ctrl_instance = WPAMonitor(wpa_supplicant_control_path)
        wpa_thread = threading.Thread(target=wpa_ctrl_instance.start_monitoring, args=(in_queue_lower,))
        wpa_thread.start()

    # Start multicast receiver that listens to multicasts from other mesh nodes
    receiver_thread = threading.Thread(target=mua.multicast_handler.receive_multicast)
    monitor_thread = threading.Thread(target=mua.monitor_wpa_multicast)
    receiver_thread.start()
    monitor_thread.start()
    # Send periodic multicasts
    mua.sender_thread.start()


def main():
    '''
    parser = argparse.ArgumentParser(description="Network access control with CBMA")
    parser.add_argument('--interface_name', required=True, help='Interface name: Eg. wlp1s0, halow1')
    parser.add_argument('--upper_flag', default=True,
                        help='Flag to set up upper macsec, batman: True or False (default: True)')
    parser.add_argument('--lan_bridge_flag', default=True,
                        help='Flag to add upper batman for this radio to lan bridge: True or False (default: True)')
    args = parser.parse_args()
    '''
    cbma_lower(interface_name="wlp1s0", path_to_certificate=f'{file_dir}/cert_generation/certificates', wpa_supplicant_control_path='/var/run/wpa_supplicant_id0/wlp1s0')

if __name__ == "__main__":
    main()