import socket
import ctypes
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy

from px4_msgs.msg import VehicleGpsPosition


# Message structure to hold vehicle GPS position data.
# Reference: https://github.com/PX4/px4_msgs/blob/master/msg/VehicleGpsPosition.msg.
class VehicleGpsPositionMsg(ctypes.Structure):
    _fields_ = [("timestamp", ctypes.c_uint64),
                ("lat", ctypes.c_int32),
                ("lon", ctypes.c_int32),
                ("alt", ctypes.c_int32),
                ("alt_ellipsoid", ctypes.c_int32),
                ("s_variance_m_s", ctypes.c_float),
                ("c_variance_rad", ctypes.c_float),
                ("fix_type", ctypes.c_uint8),
                ("eph", ctypes.c_float),
                ("epv", ctypes.c_float),
                ("hdop", ctypes.c_float),
                ("vdop", ctypes.c_float),
                ("noise_per_ms", ctypes.c_int32),
                ("jamming_indicator", ctypes.c_int32),
                ("jamming_state", ctypes.c_uint8),
                ("vel_m_s", ctypes.c_float),
                ("vel_n_m_s", ctypes.c_float),
                ("vel_e_m_s", ctypes.c_float),
                ("vel_d_m_s", ctypes.c_float),
                ("cog_rad", ctypes.c_float),
                ("vel_ned_valid", ctypes.c_bool),
                ("timestamp_time_relative", ctypes.c_int32),
                ("time_utc_usec", ctypes.c_uint64),
                ("satellites_used", ctypes.c_uint8),
                ("heading", ctypes.c_float),
                ("heading_offset", ctypes.c_float),
                ("selected", ctypes.c_uint8)]


def _cpython_packaging(msg):
    vehicleGPSMsg = VehicleGpsPositionMsg()

    # msg.<float>* are checked for 'nan' and value replaced with 0 as cpython
    # is not accepting 'nan'

    vehicleGPSMsg.timestamp = msg.timestamp
    vehicleGPSMsg.lat = msg.lat
    vehicleGPSMsg.lon = msg.lon
    vehicleGPSMsg.alt = msg.alt
    vehicleGPSMsg.alt_ellipsoid = msg.alt_ellipsoid
    vehicleGPSMsg.s_variance_m_s = msg.s_variance_m_s \
        if msg.s_variance_m_s == msg.s_variance_m_s else 0
    vehicleGPSMsg.c_variance_rad = msg.c_variance_rad \
        if msg.c_variance_rad == msg.c_variance_rad else 0
    vehicleGPSMsg.fix_type = msg.fix_type
    vehicleGPSMsg.eph = msg.eph if msg.eph == msg.eph else 0
    vehicleGPSMsg.epv = msg.epv if msg.epv == msg.epv else 0
    vehicleGPSMsg.hdop = msg.hdop if msg.hdop == msg.hdop else 0
    vehicleGPSMsg.vdop = msg.vdop if msg.vdop == msg.vdop else 0
    vehicleGPSMsg.noise_per_ms = msg.noise_per_ms
    vehicleGPSMsg.jamming_indicator = msg.jamming_indicator
    vehicleGPSMsg.jamming_state = msg.jamming_state
    vehicleGPSMsg.vel_m_s = msg.vel_m_s if msg.vel_m_s == msg.vel_m_s else 0
    vehicleGPSMsg.vel_n_m_s = msg.vel_n_m_s if msg.vel_n_m_s == msg.vel_n_m_s else 0
    vehicleGPSMsg.vel_e_m_s = msg.vel_e_m_s if msg.vel_e_m_s == msg.vel_e_m_s else 0
    vehicleGPSMsg.vel_d_m_s = msg.vel_d_m_s if msg.vel_d_m_s == msg.vel_d_m_s else 0
    vehicleGPSMsg.cog_rad = msg.cog_rad if msg.cog_rad == msg.cog_rad else 0
    vehicleGPSMsg.vel_ned_valid = msg.vel_ned_valid
    vehicleGPSMsg.timestamp_time_relative = msg.timestamp_time_relative
    vehicleGPSMsg.time_utc_usec = msg.time_utc_usec
    vehicleGPSMsg.satellites_used = msg.satellites_used
    vehicleGPSMsg.heading = msg.heading if msg.heading == msg.heading else 0
    vehicleGPSMsg.heading_offset = msg.heading_offset \
        if msg.heading_offset == msg.heading_offset else 0
    vehicleGPSMsg.selected = msg.selected

    return vehicleGPSMsg


class DRILocSubscriber(Node):
    def __init__(self):
        super().__init__('dri_location_subscriber')
        # https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html
        # need to follow publisher QoS settings in most cases
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.subscription = self.create_subscription(
            VehicleGpsPosition,
            'fmu/vehicle_gps_position/out',
            self.listener_callback,
            qos)  # use QoS
        self.subscription  # prevent unused variable warning
        self.HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
        self.PORT = 33222  # Port to listen on (non-privileged ports are > 1023)
        self.dri_loc_socket = socket.socket()  # to remove python warning
        self.backup_timer = 0
        self.backup_data = ""
        self.setup_socket()

    def setup_socket(self):
        try:
            self.get_logger().info('dri socket init')
            self.dri_loc_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.dri_loc_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.dri_loc_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.dri_loc_socket.settimeout(1)
            self.dri_loc_socket.connect((self.HOST, self.PORT))
        except socket.error as message:
            print(f"Type Error: {message}")

    def listener_callback(self, msg):
        try:
            byte_data = _cpython_packaging(msg)
            gps_msg = struct.pack('>I', ctypes.sizeof(byte_data)) + byte_data
            self.dri_loc_socket.sendall(gps_msg)
        except (ConnectionRefusedError, socket.timeout, ConnectionResetError, ConnectionRefusedError):
            self.setup_socket()


def main(args=None):
    rclpy.init(args=args)

    dri_loc_subscriber = DRILocSubscriber()

    rclpy.spin(dri_loc_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dri_loc_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
