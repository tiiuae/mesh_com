import rclpy
from rclpy.node import Node
from std_msgs.msg import String

i = ('{"api_version": 1,'
     '"ssid": "mesh",'
     '"key": "1234567890",'
     '"ap_mac": "00:11:22:33:44:55",'
     '"country": "fi",'
     '"frequency": "5220",'
     '"ip": "192.168.1.2",'
     '"subnet":  "255.255.255.0",'
     '"tx_power": "30",'
     '"mode": "mesh"}')


class MeshPublisher(Node):

    def __init__(self):
        super().__init__('mesh_publisher')
        self.publisher_ = self.create_publisher(String, 'mesh_parameters', 10)
        timer_period = 100  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = i

    def timer_callback(self):
        msg = String()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Mesh Publishing: "%s"' % msg.data)


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
