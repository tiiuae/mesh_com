import ctypes
import struct
import yaml
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from px4_msgs.msg import VehicleGpsPosition
import time

from .transport.rid_nats import ridNatsClient
from .transport.dri_broadcast import dri_broadcast
from .astm.rid_astm_f3411 import rid_astm_f3411

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

class RIDLocSubscriber(Node):
    def __init__(self):
        super().__init__('rid_location_subscriber')
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
        self.backup_timer = 0
        self.backup_data = ""
        self.yaml_file = "/opt/ros/galactic/share/mesh_com/ussp.yaml"
        self.rid_type = None
        self.rid_frequency = None
        self.rid_certfile = None
        self.rid_keyfile = None
        self.client = None

    async def listener_callback(self, msg):
        if self.rid_type == "broadcast":
            try:
                byte_data = _cpython_packaging(msg)
                gps_msg = struct.pack('>I', ctypes.sizeof(byte_data)) + byte_data
                self.client.dri_loc_socket.sendall(gps_msg)
            except (ConnectionRefusedError, socket.timeout, ConnectionResetError, ConnectionRefusedError):
                self.client.setup_socket()
        elif self.rid_type == "nats":
             jmsg = self.encoder.init_data_fields(msg.timestamp, "AirBone", msg.lat, msg.lon, msg.alt, msg.eph, msg.epv, "Unkown", "Unkown", "Unkown", "Unkown", msg.vel_m_s, msg.timestamp_time_relative, msg.vel_e_m_s, msg.vel_d_m_s)
             jmsg = self.encoder.encode_data_fields()
             self.client.loop.run_until_complete(self.client.publish_async("rid", jmsg))
             time.sleep(1/self.rid_frequency)
        else:
            print("rid transport not supported")

        # Add data tx method based on mode of transport(broadcast/network)

    def init_rid_config(self):
        with open(self.yaml_file, 'r') as f:
            parsed_yaml = yaml.safe_load(f)
            if "rid_type" in parsed_yaml:
                self.rid_type = parsed_yaml["rid_type"]

            if "rid_frequency" in parsed_yaml:
                self.rid_frequency = parsed_yaml["rid_frequency"]

            if "rid_certfile" in parsed_yaml:
                self.rid_certfile = parsed_yaml["rid_certfile"]

            if "rid_keyfile" in parsed_yaml:
                self.rid_keyfile = parsed_yaml["rid_keyfile"]

            if "rid_server" in parsed_yaml:
                self.rid_server = parsed_yaml["rid_server"]

            if "rid_port" in parsed_yaml:
                self.rid_port = parsed_yaml["rid_port"]

    def init_rid_transport(self):
        if self.rid_type == "broadcast":
            self.client = dri_broadcast(server=self.rid_server, port=self.rid_port)
        elif self.rid_type == "nats":
            self.encoder = rid_astm_f3411()
            self.client = ridNatsClient(server=self.rid_server, port=self.rid_port, certfile=self.rid_certfile, keyfile=self.rid_keyfile)
            print("NATS Server", self.rid_server, "port:", self.rid_port)
            # Connect to NATS server
            self.client.loop.run_until_complete(self.client.connect())
        else:
            print("rid transport not supported")

def main(args=None):
    rclpy.init(args=args)

    rid_loc_subscriber = RIDLocSubscriber()
    rid_loc_subscriber.init_rid_config()
    rid_loc_subscriber.init_rid_transport()
    rclpy.spin(rid_loc_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rid_loc_subscriber.destroy_node()
    rclpy.shutdown()

    if "rid_type" == "nats":
        # Close connection to NATS server
        self.client.loop.run_until_complete(self.client.close())

if __name__ == '__main__':
    main()
