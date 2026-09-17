import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class Listener(Node):

    def __init__(self):
        super().__init__('listener')

        self.subscription = self.create_subscription(
            Float32,
            '/random_number',
            self.listener_callback,
            10
        )

    def listener_callback(self, message):
        received = message.data
        multiplied = received * 2

        self.get_logger().info(
            f'Received: {received:.2f}. Multiplied value: {multiplied:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = Listener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
