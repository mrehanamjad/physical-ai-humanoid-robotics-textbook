import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ParamNode(Node):
    def __init__(self):
        super().__init__('my_hello_world_node')
        self.declare_parameter('message', 'Default Hello')
        self.declare_parameter('publish_frequency', 1.0)

        self.message = self.get_parameter('message').get_parameter_value().string_value
        self.publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
        self.timer = self.create_timer(1.0 / self.publish_frequency, self.timer_callback)
        self.get_logger().info(f'Node initialized with message: "{self.message}" and frequency: {self.publish_frequency} Hz')

    def timer_callback(self):
        msg = String()
        msg.data = self.message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = ParamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
