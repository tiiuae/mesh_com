import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from std_msgs.msg import String
import subprocess


class BatmanVisualisation:

    command = 'batadv-vis'

    def __str__(self):
        return 'BatmanVisualisation \'{0}\''.format(self.batadv_vis)

    def get(self, format_type="dot"):
        if format_type == "dot" or format_type == "jsondoc" or format_type == "json":
            try:
                # returns byte string
                raw_data = subprocess.check_output([self.command, '-f', format_type])
                if raw_data == 255:
                    raw_data = b'NA'
            except (FileNotFoundError, subprocess.CalledProcessError):
                raw_data = b'NA'
        else:
            raw_data = b'NA'
        # return string
        return raw_data.decode('UTF-8').strip('\n')


class MeshPublisher(Node):

    def __init__(self):
        super().__init__('mesh_publisher')
        self.publisher_ = self.create_publisher(String, 'mesh_visual', QoSPresetProfiles.SYSTEM_DEFAULT.value)
        timer_period = 10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.batman = BatmanVisualisation()

    def timer_callback(self):
        msg = String()
        msg.data = self.batman.get(format_type="dot")
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
