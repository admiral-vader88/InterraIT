#pubsliher in python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('py_talker')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.on_timer)
        self.count = 0
        self.get_logger().info("Python Talker started")

    def on_timer(self):
        msg = String()
        msg.data = f"Hello from Python talker: {self.count}"
        self.pub.publish(msg)
        self.count += 1

def main():
    rclpy.init()
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

# subsriber in python 
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('py_listener')
        self.sub = self.create_subscription(String, 'chatter', self.cb, 10)
        self.get_logger().info("Python Listener started")

    def cb(self, msg: String):
        self.get_logger().info(f"I heard: '{msg.data}'")

def main():
    rclpy.init()
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

