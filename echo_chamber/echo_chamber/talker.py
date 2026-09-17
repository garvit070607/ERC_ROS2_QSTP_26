import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class Talker(Node):

    def __init__(self):
        super().__init__('talker')

        self.publisher = self.create_publisher(
            Float32,
            '/random_number',
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.publish_random_number
        )

    def publish_random_number(self):
        number = random.uniform(0.0, 100.0)

        message = Float32()
        message.data = number

        self.publisher.publish(message)

        self.get_logger().info(
            f'Publishing: {number:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = Talker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

