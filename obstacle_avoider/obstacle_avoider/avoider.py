import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool


class ObstacleAvoider(Node):

    def __init__(self):
        super().__init__('obstacle_avoider')

        # Robot state: OFF by default
        self.is_active = False

        # Publisher for robot movement
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Subscriber for LiDAR data
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Service to turn robot ON/OFF
        self.toggle_service = self.create_service(
            SetBool,
            '/toggle_robot',
            self.toggle_robot
        )

        # Timer for movement
        self.timer = self.create_timer(0.1, self.control_loop)

        # Latest LiDAR data
        self.latest_scan = None

        # Used to slowly change the spiral
        self.angular_speed = 0.5

        self.get_logger().info('Obstacle Avoider started. Robot is OFF.')

    # --------------------------------------------------
    # Service: /toggle_robot
    # --------------------------------------------------

    def toggle_robot(self, request, response):

        self.is_active = request.data

        if self.is_active:
            response.success = True
            response.message = 'Robot turned ON.'
            self.get_logger().info('Robot turned ON.')
        else:
            response.success = True
            response.message = 'Robot turned OFF.'
            self.stop_robot()
            self.get_logger().info('Robot turned OFF.')

        return response

    # --------------------------------------------------
    # LiDAR callback
    # --------------------------------------------------

    def scan_callback(self, msg):

        self.latest_scan = msg

    # --------------------------------------------------
    # Main control loop
    # --------------------------------------------------

    def control_loop(self):

        # If robot is OFF, keep it stopped
        if not self.is_active:
            self.stop_robot()
            return

        # Wait until LiDAR data arrives
        if self.latest_scan is None:
            return

        scan = self.latest_scan

        # Get front, left and right distances
        front = scan.ranges[0]
        left = scan.ranges[90]
        right = scan.ranges[270]

        cmd = Twist()

        # --------------------------------------------------
        # Obstacle detected
        # --------------------------------------------------

        if front < 1.0:

            # Stop moving forward
            cmd.linear.x = 0.0

            # Turn toward the more open side
            if left > right:
                cmd.angular.z = 0.8
                self.get_logger().info('Obstacle ahead -> turning LEFT')
            else:
                cmd.angular.z = -0.8
                self.get_logger().info('Obstacle ahead -> turning RIGHT')

        # --------------------------------------------------
        # No close obstacle -> spiral
        # --------------------------------------------------

        else:

            cmd.linear.x = 0.2
            cmd.angular.z = self.angular_speed

            # Slowly decrease angular velocity
            self.angular_speed -= 0.002

            # Don't let it become zero
            if self.angular_speed < 0.1:
                self.angular_speed = 0.1

        self.cmd_vel_publisher.publish(cmd)

    # --------------------------------------------------
    # Stop robot
    # --------------------------------------------------

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_vel_publisher.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = ObstacleAvoider()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.stop_robot()
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
