import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from irobot_create_msgs.msg import LightringLeds, LedColor
from turtlebot4_msgs.msg import UserLed
from oslo_log import log as logging
from iotronic_lightningrod.modules.plugins import Plugin
from std_msgs.msg import String

LOG = logging.getLogger(__name__)

def log_info(msg):
    try:
        LOG.info(msg)
    except Exception:
        print("[INFO]", msg)

def log_error(msg):
    try:
        LOG.error(msg)
    except Exception:
        print("[ERROR]", msg)

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, '/example_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_message) 
        self.counter = 0

    def publish_message(self):
        msg = String()
        msg.data = f"Hello, ROS 2! Count: {self.counter}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: '{msg.data}'")
        self.counter += 1

class Worker(Plugin.Plugin):
    def __init__(self, uuid, name, q_result=None, params=None):
        super(Worker, self).__init__(uuid, name, q_result, params)

    def run(self):
        rclpy.init()
        simple_publisher = SimplePublisher()
        try:
            rclpy.spin(simple_publisher)
        except Exception as e:
            LOG.error('Error occurred while spinning node: %s', e)
        finally:
            self.stop()
