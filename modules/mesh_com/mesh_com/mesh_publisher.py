import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from std_msgs.msg import String
import socket


class MeshPublisher(Node):
    def __init__(self):
        super().__init__('mesh_publisher')
        self.publisher_ = self.create_publisher(String, 'mesh_visual',
                                                QoSPresetProfiles.SYSTEM_DEFAULT.value)
        timer_period = 10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
        self.PORT = 33221  # Port to listen on (non-privileged ports are > 1023)
        self.mesh_socket = 0
        self.backup_timer = 0

    def timer_callback(self):
        msg = String()

        try:
            self.destroy_timer(self.backup_timer)
            self.setup_socket()
            self.mesh_socket.sendall(str.encode("report"))
            data = self.mesh_socket.recv(2048)
            self.mesh_socket.close()
            msg.data = data.decode('utf-8')
            self.publisher_.publish(msg)
            self.get_logger().info('Mesh Publishing: "%s"' % msg.data)
        except:
            self.backup_timer = self.create_timer(
                1.0, self.backup_caller)

    def setup_socket(self):
        self.mesh_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.mesh_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.mesh_socket.settimeout(1)

        self.mesh_socket.connect((self.HOST, self.PORT))

    def backup_caller(self):
        msg = String()

        try:
            self.destroy_timer(self.backup_timer)
            self.get_logger().info('publisher backup_caller"')
            self.setup_socket()
            self.mesh_socket.sendall(str.encode("report"))
            data = self.mesh_socket.recv(2048)
            self.mesh_socket.close()
            msg.data = data.decode('utf-8')
            self.publisher_.publish(msg)
            self.get_logger().info('Mesh Publishing: "%s"' % msg.data)
        except:
            self.backup_timer = self.create_timer(
                1.0, self.backup_caller)


def main(args=None):
    rclpy.init(args=args)

    mesh_publisher = MeshPublisher()

    rclpy.spin(mesh_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mesh_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
