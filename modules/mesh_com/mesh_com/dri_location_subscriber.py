import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from std_msgs.msg import String
import socket

from .src.socket_helper import send_msg
from px4_msgs . msg import SensorGps

class DRILocSubscriber(Node):
    def __init__(self):
        super().__init__('dri_loc_subscriber')
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        if __MOCAP__:
            self.subscription = self.create_subscription(
                String,
                'uaeci01/fmu/sensor_gps/in',
                self.listener_callback,
                qos)
        else:
            self.subscription = self.create_subscription(
                String,
                'fmu/vehicle_gps_position/out',
                self.listener_callback,
                qos)
        self.subscription  # prevent unused variable warning
        self.HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
        self.PORT = 33222  # Port to listen on (non-privileged ports are > 1023)
        self.dri_loc_socket = socket.socket()  # to remove python warning
        self.backup_timer = 0
        self.backup_data = ""

    def setup_socket(self):
        self.dri_loc_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.dri_loc_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.dri_loc_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.dri_loc_socket.settimeout(1)

    def extract_loc_data(self):
        #https://github.com/PX4/px4_msgs/blob/master/msg/SensorGps.msg

    def listener_callback(self, msg):
        self.get_logger().info('listener_callback')
        self.destroy_timer(self.backup_timer)  # resend fresh data only
        self.get_logger().info('mesh dri location subscriber: "%s"' % msg.data)
        try:
            if msg.data:
                self.setup_socket()
                self.dri_loc_socket.connect((self.HOST, self.PORT))
                send_msg(self.dri_loc_socket, str.encode(msg.data))
                self.dri_loc_socket.close()
        except (ConnectionRefusedError, socket.timeout, ConnectionResetError):
            self.backup_data = msg.data
            self.backup_timer = self.create_timer(
                2.0, self.backup_caller)

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
