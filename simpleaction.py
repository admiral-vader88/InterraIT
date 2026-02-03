# simple action for python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddClient(Node):
    def __init__(self):
        super().__init__('py_add_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service...")
        self.get_logger().info("Service found!")

    def call(self, a, b):
        req = AddTwoInts.Request()
        req.a = a
        req.b = b
        future = self.cli.call_async(req)
        return future

def main():
    rclpy.init()
    node = AddClient()
    future = node.call(10, 20)
    rclpy.spin_until_future_complete(node, future)
    if future.result():
        node.get_logger().info(f"Result: {future.result().sum}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

# action client for python 
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci

class FibClient(Node):
    def __init__(self):
        super().__init__('py_fib_action_client')
        self.client = ActionClient(self, Fibonacci, 'fib')

    def send_goal(self, order=10):
        goal = Fibonacci.Goal()
        goal.order = order
        self.client.wait_for_server()
        return self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)

    def feedback_cb(self, feedback_msg):
        seq = feedback_msg.feedback.sequence
        self.get_logger().info(f"Feedback: {seq}")

def main():
    rclpy.init()
    node = FibClient()
    future_goal = node.send_goal(8)

    rclpy.spin_until_future_complete(node, future_goal)
    goal_handle = future_goal.result()
    if not goal_handle.accepted:
        node.get_logger().info("Goal rejected")
        return

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)

    node.get_logger().info(f"Result: {result_future.result().result.sequence}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

