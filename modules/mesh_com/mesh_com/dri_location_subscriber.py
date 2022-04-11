import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from std_msgs.msg import String
import socket

from .src.socket_helper import send_msg
from px4_msgs.msg import VehicleGpsPosition


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
        self.dri_loc_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.dri_loc_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.dri_loc_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.dri_loc_socket.settimeout(1)
        self.dri_loc_socket.connect((self.HOST, self.PORT))

    def listener_callback(self, msg):
        self.get_logger().info('listener_callback')
        self.destroy_timer(self.backup_timer)  # resend fresh data only
        #self.get_logger().info('mesh dri location subscriber: "%s"' % msg)
        # refer to https://github.com/PX4/px4_msgs/blob/master/msg/SensorGps.msg
        gps_msg = (f"{int(msg.lat)/10000000};{int(msg.lon)/10000000};")
        self.get_logger().info(gps_msg)
        # gps_msg=(str(msg.timestamp) + ';' + str(msg.device_id) + ';' + str(msg.lat) + ';' + str(msg.lon) + ';' + str(msg.alt) + ';' +
        #         str(msg.alt_ellipsoid) + ';' + str(msg.s_variance_m_s) + ';' + str(msg.c_variance_rad) + ';' + str(msg.fix_type) + ';' + str(msg.eph) + ';' +
        #         str(msg.epv) + ';' + str(msg.hdop) + ';' + str(msg.vdop) + ';' + str(msg.jamming_indicator) + ';' + str(msg.jamming_state) + ';' +
        #         str(msg.epv) + ';' + str(msg.hdop) + ';' + str(msg.vdop) + ';' + str(msg.jamming_indicator) + ';' + str(msg.jamming_state) + ';' +
        #         str(msg.vel_m_s) + ';' + str(msg.vel_n_m_s) + ';' + str(msg.vel_e_m_s) + ';' + str(msg.vel_d_m_s) + ';' + str(msg.timestamp_time_relative) + ';' +
        #         str(msg.time_utc_usec) + ';' + str(msg.satellites_used) + ';' + str(msg.heading) + ';' + str(msg.heading_offset) + ';' + str(msg.heading_accuracy) + ';')

        try:
            if gps_msg:
                send_msg(self.dri_loc_socket, str.encode(gps_msg))
                self.get_logger().info('sent')
        except (ConnectionRefusedError, socket.timeout, ConnectionResetError):
            self.get_logger().info('exception')
            #self.backup_data = gps_msg
            #self.backup_timer = self.create_timer(
            #    2.0, self.backup_caller)

    def backup_caller(self):
        self.get_logger().info('backup_caller: "%s"' % self.backup_data)
        try:
            self.destroy_timer(self.backup_timer)
            self.setup_socket()
            self.dri_loc_socket.connect((self.HOST, self.PORT))
            send_msg(self.dri_loc_socket, str.encode(self.backup_data))
            self.dri_loc_socket.close()
        except (ConnectionRefusedError, socket.timeout, ConnectionResetError):
            self.get_logger().info('dri loc backup_caller: ##')
            self.backup_timer = self.create_timer(
                2.0, self.backup_caller)


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
