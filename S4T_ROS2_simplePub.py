import rclpy
from rclpy.node import Node
from rclpy.utilities import *
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from oslo_log import log as logging
from iotronic_lightningrod.modules.plugins import Plugin
from std_msgs.msg import String


class InfoPublisher(Node):
    def __init__(self):
        super().__init__('info_publisher')

        self.pub = self.create_publisher(String, '/example_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello ROS 2! Count: {self.counter}"
        self.pub.publish(msg)
        print(f"PUBLISH: '{msg.data}'")
        self.counter += 1

class Worker(Plugin.Plugin):
    def __init__(self, uuid, name, q_result=None, params=None):
        super(Worker, self).__init__(uuid, name, q_result, params)


    def run(self):
        rclpy.init()
        node = InfoPublisher()
        self.q_result.put("ROS2_simplePublisher plugin correctly started") # Used to notify the correct start of the plugin to S4T
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
            self.stop()
