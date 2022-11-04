import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from std_msgs.msg import String
import socket
import signal

from .src.socket_helper import recv_msg, send_msg


class MeshPublisher(Node):
    def __init__(self):
        super().__init__('mesh_publisher')
        self.publisher_ = self.create_publisher(String, 'mesh_visual',
                                                QoSPresetProfiles.SYSTEM_DEFAULT.value)
        timer_period = 10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
        self.PORT = 33221  # Port to listen on (non-privileged ports are > 1023)
        self.mesh_socket = socket.socket()  # to remove python warning

    def timer_callback(self):
        msg = String()
        self.get_logger().info('publisher timer_callback')
        try:
            self.setup_socket()
            self.mesh_socket.connect((self.HOST, self.PORT))
            send_msg(self.mesh_socket, str.encode("report\n"))
            data = recv_msg(self.mesh_socket)
            self.mesh_socket.close()
            if data:
                msg.data = data.decode('utf-8')
                self.publisher_.publish(msg)
            # self.get_logger().info('Mesh Publishing: "%s"' % msg.data)
        except (ConnectionRefusedError, socket.timeout, ConnectionResetError):
            pass

    def setup_socket(self):
        self.mesh_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.mesh_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.mesh_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.mesh_socket.settimeout(1)

# Python does not register SIGINT if it is not called from an interactive shell.
# So we need our own signal handler.
def sigint_handler(_signo, _stack_frame):
    raise KeyboardInterrupt

def main(args=None):
    signal.signal(signal.SIGINT, sigint_handler)

    rclpy.init(args=args)

    mesh_publisher = MeshPublisher()

    try:
        rclpy.spin(mesh_publisher)
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        print('mesh_publisher.destroy_node')
        mesh_publisher.destroy_node()
        print('rclpy.shutdown')
        rclpy.shutdown()


if __name__ == '__main__':
    main()
