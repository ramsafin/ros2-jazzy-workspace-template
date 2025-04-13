import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')

        self.counter = 0
        self.publish_rate_hz = 2
        self.qos_history_depth = 10

        self.publisher_ = self.create_publisher(String, 'topic', self.qos_history_depth)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'

        self.publisher_.publish(msg)
        self.counter += 1

        self.get_logger().info(f'Publishing: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    publisher = MinimalPublisher()
    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
