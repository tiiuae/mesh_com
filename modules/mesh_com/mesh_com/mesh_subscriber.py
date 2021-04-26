import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from std_msgs.msg import String

import json
import subprocess
from shlex import quote


class MeshSubscriber(Node):
    class MeshSettings:
        def __init__(self):
            self.api_version = 1
            self.ssid = ""
            self.key = ""
            self.ap_mac = ""
            self.country = ""
            self.frequency = ""
            self.ip = ""
            self.subnet = ""
            self.tx_power = ""
            self.mode = ""
            # self.enc = ""

    def __init__(self):
        super().__init__('mesh_subscriber')
        self.subscription = self.create_subscription(
            String,
            'mesh_parameters',
            self.listener_callback,
            QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self.subscription  # prevent unused variable warning
        self.settings = self.MeshSettings()

    def listener_callback(self, msg):
        self.get_logger().info('mesh subscriber: "%s"' % msg.data)
        if msg.data:
            self.__handle_msg(msg.data)

    def __handle_msg(self, msg):

        # {
        #     "api_version": 1,                 interface version for future purposes
        #     "ssid": "gold",                   0-32 octets, UTF-8, shlex.quote chars limiting
        #     "key": "foobar",                  key for the network
        #     "ap_mac": "00:11:22:33:44:55",    bssid for mesh network
        #     "country": "UA",                  Country code, sets tx power limits and supported
        #                                       channels
        #     "frequency": "5220",              wifi channel frequency, depends on the country
        #                                       code and HW
        #     "ip": "192.168.1.1",              select unique IP
        #     "subnet": "255.255.255.0",
        #     "tx_power": "30",                 select 30dBm, HW and regulations limiting it
        #                                       correct level.
        #                                       Can be used to set lower dBm levels for testing
        #                                       purposes (e.g. 5dBm)
        #     "mode": "mesh"                    mesh=mesh network, ap=debug hotspot
        # }

        try:
            self.get_logger().info('Before JSON loads')
            parameters = json.loads(msg)
            self.get_logger().info('After JSON loads')
            self.settings.ssid = parameters["ssid"]
            self.settings.key = parameters["key"]
            self.settings.ap_mac = parameters["ap_mac"]
            self.settings.country = parameters["country"].lower()
            self.settings.frequency = parameters["frequency"]
            self.settings.ip = parameters["ip"]
            self.settings.subnet = parameters["subnet"]
            self.settings.tx_power = parameters["tx_power"]
            self.settings.mode = parameters["mode"]
            # self.settings.enc = parameters["enc"]
            self.get_logger().info('Enter __change_configuration()')
            self.__change_configuration()
        except json.decoder.JSONDecodeError or KeyError or Exception:
            self.get_logger().info('Setting Failed')
            pass

    def __change_configuration(self):
        subprocess.call(["/opt/ros/foxy/share/bin/mesh-ibss.sh", quote(self.settings.mode),
                         quote(self.settings.ip),
                         quote(self.settings.subnet),
                         quote(self.settings.ap_mac),
                         quote(self.settings.key),
                         quote(self.settings.ssid),
                         quote(self.settings.frequency),
                         quote(self.settings.tx_power),
                         quote(self.settings.country)])
        self.get_logger().info('Setting Done')


def main(args=None):
    rclpy.init(args=args)

    mesh_subscriber = MeshSubscriber()

    rclpy.spin(mesh_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mesh_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
