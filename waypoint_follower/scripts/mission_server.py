#!/usr/bin/env python3

import math
import os
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory

from waypoint_follower.action import Mission


class MissionServer(Node):

    def __init__(self):
        super().__init__('mission_server')

        # Allow the /odom callback to run while the mission is executing.
        self.callback_group = ReentrantCallbackGroup()

        # Robot state from /odom
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False

        # Distance travelled by the robot
        self.total_distance = 0.0
        self.previous_x = None
        self.previous_y = None

        # Publisher for robot velocity
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Subscriber for robot position and orientation
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
            callback_group=self.callback_group
        )

        # Action server
        self.action_server = ActionServer(
            self,
            Mission,
            'follow_mission',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('Mission Server is ready.')

    # ---------------------------------------------------------
    # ODOM CALLBACK
    # ---------------------------------------------------------

    def odom_callback(self, msg):

        # Current position
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y

        # Calculate distance travelled since previous odometry update
        if self.previous_x is not None and self.previous_y is not None:
            dx = new_x - self.previous_x
            dy = new_y - self.previous_y

            self.total_distance += math.sqrt(dx * dx + dy * dy)

        self.previous_x = new_x
        self.previous_y = new_y

        self.x = new_x
        self.y = new_y

        # Convert quaternion orientation to yaw
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        sin_yaw = 2.0 * (qw * qz + qx * qy)
        cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)

        self.yaw = math.atan2(sin_yaw, cos_yaw)

        self.odom_received = True

    # ---------------------------------------------------------
    # GOAL CALLBACK
    # ---------------------------------------------------------

    def goal_callback(self, goal_request):

        self.get_logger().info(
            f'Received mission request: {goal_request.mission_file}'
        )

        return GoalResponse.ACCEPT

    # ---------------------------------------------------------
    # CANCEL CALLBACK
    # ---------------------------------------------------------

    def cancel_callback(self, goal_handle):

        self.get_logger().info('Received cancel request.')

        return CancelResponse.ACCEPT

    # ---------------------------------------------------------
    # HELPER FUNCTIONS
    # ---------------------------------------------------------

    def normalize_angle(self, angle):
        """
        Convert an angle to the range [-pi, pi].
        """

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def stop_robot(self):
        """
        Immediately stop the robot.
        """

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.cmd_vel_publisher.publish(cmd)

    def drive_to_point(self, goal_handle, target_x, target_y, waypoint_index, total_waypoints):
        """
        Drive the robot toward one target point using a simple
        proportional controller.

        Returns:
            True  -> target reached
            False -> mission cancelled
        """

        while rclpy.ok():

            # -------------------------------------------------
            # Check for cancellation
            # -------------------------------------------------

            if goal_handle.is_cancel_requested:

                self.stop_robot()

                self.get_logger().info(
                    'Mission cancelled. Robot stopped.'
                )

                goal_handle.canceled()

                return False

            # -------------------------------------------------
            # Calculate distance to target
            # -------------------------------------------------

            dx = target_x - self.x
            dy = target_y - self.y

            distance = math.sqrt(dx * dx + dy * dy)

            # -------------------------------------------------
            # Check if target has been reached
            # -------------------------------------------------

            if distance < 0.15:

                self.stop_robot()

                return True

            # -------------------------------------------------
            # Calculate desired heading
            # -------------------------------------------------

            target_angle = math.atan2(dy, dx)

            angle_error = self.normalize_angle(
                target_angle - self.yaw
            )

            # -------------------------------------------------
            # Proportional controller
            # -------------------------------------------------

            cmd = Twist()

            # If the robot is facing significantly away from
            # the target, turn first.
            if abs(angle_error) > 0.25:

                cmd.linear.x = 0.0

                # Proportional angular control
                cmd.angular.z = 1.5 * angle_error

                # Limit angular velocity
                if cmd.angular.z > 1.0:
                    cmd.angular.z = 1.0

                if cmd.angular.z < -1.0:
                    cmd.angular.z = -1.0

                status = (
                    f' turning to waypoint '
                    f'{waypoint_index}/{total_waypoints}'
                )

            else:

                # Once roughly facing the target, move forward.
                cmd.angular.z = 1.0 * angle_error

                # Proportional linear velocity
                cmd.linear.x = 0.5 * distance

                # Limit linear velocity
                if cmd.linear.x > 0.25:
                    cmd.linear.x = 0.25

                # Don't move extremely slowly near the target.
                if cmd.linear.x < 0.05:
                    cmd.linear.x = 0.05

                status = (
                    f'en route to waypoint '
                    f'{waypoint_index}/{total_waypoints}'
                )

            # -------------------------------------------------
            # Publish velocity
            # -------------------------------------------------

            self.cmd_vel_publisher.publish(cmd)

            # -------------------------------------------------
            # Send feedback
            # -------------------------------------------------

            feedback = Mission.Feedback()

            feedback.current_waypoint_index = waypoint_index
            feedback.status = status
            feedback.distance_to_target = float(distance)

            goal_handle.publish_feedback(feedback)

            # -------------------------------------------------
            # Control loop timing
            # -------------------------------------------------

            time.sleep(0.1)

        self.stop_robot()

        return False

    # ---------------------------------------------------------
    # ACTION EXECUTE CALLBACK
    # ---------------------------------------------------------

    def execute_callback(self, goal_handle):

        self.get_logger().info('Starting mission.')

        # -----------------------------------------------------
        # Load mission file
        # -----------------------------------------------------

        mission_file = goal_handle.request.mission_file

        mission_path = os.path.join(
            get_package_share_directory('waypoint_follower'),
            'missions',
            mission_file
        )

        self.get_logger().info(
            f'Loading mission file: {mission_path}'
        )

        try:

            with open(mission_path, 'r') as file:
                mission = yaml.safe_load(file)

        except Exception as error:

            self.get_logger().error(
                f'Failed to load mission file: {error}'
            )

            result = Mission.Result()
            result.success = False
            result.total_distance = 0.0
            result.waypoints_completed = 0

            goal_handle.abort()

            return result

        # -----------------------------------------------------
        # Extract mission information
        # -----------------------------------------------------

        try:

            base = mission['base']
            waypoints = mission['waypoints']
            return_to_base = mission.get('return_to_base', False)

            base_x = float(base['x'])
            base_y = float(base['y'])

        except Exception as error:

            self.get_logger().error(
                f'Invalid mission file: {error}'
            )

            result = Mission.Result()
            result.success = False
            result.total_distance = 0.0
            result.waypoints_completed = 0

            goal_handle.abort()

            return result

        # -----------------------------------------------------
        # Wait for odometry
        # -----------------------------------------------------

        self.get_logger().info('Waiting for odometry...')

        while not self.odom_received and rclpy.ok():

            if goal_handle.is_cancel_requested:

                self.stop_robot()
                goal_handle.canceled()

                result = Mission.Result()
                result.success = False
                result.total_distance = 0.0
                result.waypoints_completed = 0

                return result

            time.sleep(0.1)

        # -----------------------------------------------------
        # Reset distance counter for this mission
        # -----------------------------------------------------

        mission_start_distance = self.total_distance

        waypoints_completed = 0
        total_waypoints = len(waypoints)

        self.get_logger().info(
            f'Mission contains {total_waypoints} waypoints.'
        )

        # -----------------------------------------------------
        # Visit each waypoint in order
        # -----------------------------------------------------

        for index, waypoint in enumerate(waypoints):

            target_x = float(waypoint['x'])
            target_y = float(waypoint['y'])

            waypoint_number = index + 1

            self.get_logger().info(
                f'Going to waypoint '
                f'{waypoint_number}/{total_waypoints}: '
                f'({target_x}, {target_y})'
            )

            reached = self.drive_to_point(
                goal_handle,
                target_x,
                target_y,
                waypoint_number,
                total_waypoints
            )

            # -------------------------------------------------
            # Mission was cancelled
            # -------------------------------------------------

            if not reached:

                result = Mission.Result()

                result.success = False
                result.total_distance = float(
                    self.total_distance - mission_start_distance
                )
                result.waypoints_completed = waypoints_completed

                return result

            # -------------------------------------------------
            # Waypoint successfully reached
            # -------------------------------------------------

            waypoints_completed += 1

            self.get_logger().info(
                f'Reached waypoint '
                f'{waypoint_number}/{total_waypoints}.'
            )

        # -----------------------------------------------------
        # Return to base if requested
        # -----------------------------------------------------

        if return_to_base:

            self.get_logger().info(
                'All waypoints completed. Returning to base.'
            )

            reached_base = self.drive_to_point(
                goal_handle,
                base_x,
                base_y,
                total_waypoints + 1,
                total_waypoints + 1
            )

            # -------------------------------------------------
            # Cancelled while returning to base
            # -------------------------------------------------

            if not reached_base:

                result = Mission.Result()

                result.success = False
                result.total_distance = float(
                    self.total_distance - mission_start_distance
                )
                result.waypoints_completed = waypoints_completed

                return result

            self.get_logger().info('Returned to base.')

        # -----------------------------------------------------
        # Mission completed successfully
        # -----------------------------------------------------

        self.stop_robot()

        mission_distance = (
            self.total_distance - mission_start_distance
        )

        result = Mission.Result()

        result.success = True
        result.total_distance = float(mission_distance)
        result.waypoints_completed = waypoints_completed

        goal_handle.succeed()

        self.get_logger().info(
            'Mission completed successfully.'
        )

        self.get_logger().info(
            f'Total distance: {mission_distance:.2f} m'
        )

        self.get_logger().info(
            f'Waypoints completed: {waypoints_completed}'
        )

        return result


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = MissionServer()

    # Use multiple threads so that:
    #
    # Thread 1 -> mission execution
    # Thread 2 -> /odom callback
    #
    executor = MultiThreadedExecutor(num_threads=2)

    executor.add_node(node)

    try:

        executor.spin()

    except KeyboardInterrupt:

        node.get_logger().info(
            'Keyboard interrupt received.'
        )

    finally:

        node.stop_robot()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
