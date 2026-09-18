#!/usr/bin/env python3

import signal
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions

from waypoint_follower.action import Mission


class MissionClient(Node):

    def __init__(self):
        super().__init__('mission_client')

        self.action_client = ActionClient(
            self,
            Mission,
            'follow_mission'
        )

        self.declare_parameter(
            'mission_file',
            'mission_square.yaml'
        )

        self.goal_handle = None
        self.result_future = None
        self.cancel_requested = False
        self.finished = False

    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback

        self.get_logger().info(
            f'Waypoint {feedback.current_waypoint_index} | '
            f'{feedback.status} | '
            f'Distance: {feedback.distance_to_target:.2f} m'
        )

    def send_goal(self):

        mission_file = self.get_parameter(
            'mission_file'
        ).get_parameter_value().string_value

        self.get_logger().info(
            f'Waiting for action server...'
        )

        if not self.action_client.wait_for_server(
            timeout_sec=5.0
        ):
            self.get_logger().error(
                'Action server not available.'
            )
            self.finished = True
            return

        self.get_logger().info(
            f'Sending mission: {mission_file}'
        )

        goal_msg = Mission.Goal()
        goal_msg.mission_file = mission_file

        future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):

        try:
            self.goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(
                f'Failed to send goal: {e}'
            )
            self.finished = True
            return

        if not self.goal_handle.accepted:

            self.get_logger().error(
                'Mission goal was rejected.'
            )

            self.finished = True
            return

        self.get_logger().info(
            'Mission goal accepted.'
        )

        self.result_future = (
            self.goal_handle.get_result_async()
        )

        self.result_future.add_done_callback(
            self.result_callback
        )

    def result_callback(self, future):

        try:
            result = future.result().result

            self.get_logger().info(
                f'Mission finished | '
                f'Success: {result.success} | '
                f'Waypoints completed: '
                f'{result.waypoints_completed} | '
                f'Total distance: '
                f'{result.total_distance:.2f} m'
            )

        except Exception as e:

            self.get_logger().error(
                f'Failed to get mission result: {e}'
            )

        self.finished = True

    def request_cancel(self):

        if self.cancel_requested:
            return

        self.cancel_requested = True

        if self.goal_handle is None:

            self.get_logger().warn(
                'No active goal to cancel.'
            )

            self.finished = True
            return

        self.get_logger().info(
            'Ctrl+C detected. Requesting mission cancellation...'
        )

        future = self.goal_handle.cancel_goal_async()

        future.add_done_callback(
            self.cancel_response_callback
        )

    def cancel_response_callback(self, future):

        try:

            cancel_response = future.result()

            if len(cancel_response.goals_canceling) > 0:

                self.get_logger().info(
                    'Cancellation request accepted by server.'
                )

            else:

                self.get_logger().warn(
                    'Cancellation request was not accepted.'
                )

        except Exception as e:

            self.get_logger().error(
                f'Cancellation request failed: {e}'
            )


def main():

    # Prevent ROS 2 from automatically shutting down rclpy
    # when Ctrl+C is pressed. We handle Ctrl+C ourselves.
    rclpy.init(
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = MissionClient()

    interrupted = False

    def handle_sigint(signum, frame):

        nonlocal interrupted

        if not interrupted:

            interrupted = True
            node.request_cancel()

    signal.signal(
        signal.SIGINT,
        handle_sigint
    )

    node.send_goal()

    try:

        while rclpy.ok() and not node.finished:

            rclpy.spin_once(
                node,
                timeout_sec=0.1
            )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
