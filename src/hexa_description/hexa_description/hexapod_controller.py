#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class ContinuousHexapodGait(Node):
    def __init__(self):
        super().__init__('continuous_hexapod_gait')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            'joint_trajectory_controller/follow_joint_trajectory'
        )
        self.get_logger().info('Waiting for the action server...')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available! Shutting down...')
            rclpy.shutdown()
        else:
            self.get_logger().info('Action server available.')

        # Create a timer that sends a new gait cycle every 9 seconds
        # (Our gait cycle lasts 8 seconds; the extra second provides a slight buffer.)
        self._gait_timer = self.create_timer(9.0, self.send_gait)

    def send_gait(self):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            "Revolute 1", "Revolute 2", "Revolute 3",
            "Revolute 4", "Revolute 5", "Revolute 6",
            "Revolute 7", "Revolute 8", "Revolute 9",
            "Revolute 10", "Revolute 11", "Revolute 12",
            "Revolute 13", "Revolute 14", "Revolute 15",
            "Revolute 16", "Revolute 17", "Revolute 18"
        ]
        points = []

        # Neutral configuration (kept within joint limits as defined in hexa.xacro)
        neutral = [
            0.0,   -0.5, 0.0,    # Leg 1: hip, knee, ankle
            0.0,   -0.5, 0.0,    # Leg 2
            0.0,   -0.5, 0.0,    # Leg 3
            0.0,    0.5, 0.0,    # Leg 4
            0.0,    0.5, 0.0,    # Leg 5
            0.0,   -0.5, 0.0     # Leg 6
        ]
        p0 = JointTrajectoryPoint()
        p0.positions = neutral
        p0.time_from_start = Duration(sec=2, nanosec=0)
        points.append(p0)

        # Point 1: Phase 1 – Group A (Legs 1,3,5) swing; Group B (Legs 2,4,6) push off
        p1_positions = [
             0.5,  -1.0,  0.3,    # Leg 1: swing (hip forward, knee flexed, ankle lifted)
            -0.3,  -0.5,  0.0,    # Leg 2: stance/push-off
             0.5,  -0.9,  0.3,    # Leg 3: swing
            -0.3,   0.5,  0.0,    # Leg 4: stance
             0.5,   0.8,  0.3,    # Leg 5: swing
            -0.3,  -0.5,  0.0     # Leg 6: stance
        ]
        p1 = JointTrajectoryPoint()
        p1.positions = p1_positions
        p1.time_from_start = Duration(sec=4, nanosec=0)
        points.append(p1)

        # Point 2: Phase 2 – Group B swings; Group A provides support
        p2_positions = [
            -0.3,  -0.5,  0.0,    # Leg 1: stance
             0.5,  -1.0,  0.3,    # Leg 2: swing
            -0.3,  -0.5,  0.0,    # Leg 3: stance
             0.5,   0.8,  0.3,    # Leg 4: swing
            -0.3,   0.5,  0.0,    # Leg 5: stance
             0.5,  -1.0,  0.3     # Leg 6: swing
        ]
        p2 = JointTrajectoryPoint()
        p2.positions = p2_positions
        p2.time_from_start = Duration(sec=6, nanosec=0)
        points.append(p2)

        # Point 3: Return to neutral stance
        p3 = JointTrajectoryPoint()
        p3.positions = neutral
        p3.time_from_start = Duration(sec=8, nanosec=0)
        points.append(p3)

        trajectory.points = points
        goal_msg.trajectory = trajectory

        self.get_logger().info('Sending gait trajectory goal...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.gait_goal_callback)

    def gait_goal_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gait goal rejected.')
        else:
            self.get_logger().info('Gait goal accepted.')
            # Optionally, register a callback for the result.
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.gait_result_callback)

    def gait_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Gait cycle completed with result: %s' % str(result))

def main(args=None):
    rclpy.init(args=args)
    node = ContinuousHexapodGait()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
