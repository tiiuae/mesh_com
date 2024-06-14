import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from std_msgs.msg import String
import socket
import signal

from .src.socket_helper import send_msg


class MeshSubscriber(Node):
    def __init__(self):
        super().__init__('mesh_subscriber')
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.subscription = self.create_subscription(
            String,
            'mesh_parameters',
            self.listener_callback,
            qos)
        self.subscription  # prevent unused variable warning
        self.HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
        self.PORT = 33221  # Port to listen on (non-privileged ports are > 1023)
        self.mesh_socket = socket.socket()  # to remove python warning
        self.backup_timer = 0
        self.backup_data = ""

    def setup_socket(self):
        self.mesh_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.mesh_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.mesh_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.mesh_socket.settimeout(1)

    def listener_callback(self, msg):
        self.get_logger().info('listener_callback')
        self.destroy_timer(self.backup_timer)  # resend fresh data only
        self.get_logger().info('mesh subscriber: "%s"' % msg.data)
        try:
            if msg.data:
                self.setup_socket()
                self.mesh_socket.connect((self.HOST, self.PORT))
                send_msg(self.mesh_socket, str.encode(msg.data))
                self.mesh_socket.close()
        except (ConnectionRefusedError, socket.timeout, ConnectionResetError):
            self.backup_data = msg.data
            self.backup_timer = self.create_timer(
                2.0, self.backup_caller)

    def backup_caller(self):
        self.get_logger().info('backup_caller: "%s"' % self.backup_data)
        try:
            self.destroy_timer(self.backup_timer)
            self.setup_socket()
            self.mesh_socket.connect((self.HOST, self.PORT))
            send_msg(self.mesh_socket, str.encode(self.backup_data))
            self.mesh_socket.close()
        except (ConnectionRefusedError, socket.timeout, ConnectionResetError):
            self.get_logger().info('mesh backup_caller: ##')
            self.backup_timer = self.create_timer(
                2.0, self.backup_caller)

# Python does not register SIGINT if it is not called from an interactive shell.
# So we need our own signal handler.
def sigint_handler(_signo, _stack_frame):
    raise KeyboardInterrupt

def main(args=None):
    signal.signal(signal.SIGINT, sigint_handler)

    rclpy.init(args=args)

    mesh_subscriber = MeshSubscriber()

    try:
        rclpy.spin(mesh_subscriber)
    except KeyboardInterrupt:
        # Hide the KeyboardInterrupt exception not handled.
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        print('INFO: mesh_subscriber.destroy_node')
        mesh_subscriber.destroy_node()
        print('INFO: rclpy.shutdown')
        rclpy.shutdown()
        print('INFO: Subscriber node shutdown gracefully.')


if __name__ == '__main__':
    main()
