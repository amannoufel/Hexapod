import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MultiJointController(Node):
    def __init__(self):
        super().__init__("hexapod_walker")
        # Publisher for the joint trajectory controller
        self.publisher = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )

        # Timer to periodically publish the command
        self.timer = self.create_timer(2.0, self.publish_trajectory)

    def publish_trajectory(self):
        # Create a trajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [
            "Revolute 2",  
        ]

        # Create trajectory points
        point1 = JointTrajectoryPoint()
        point1.positions = [-0.175]  # First desired positions
        point1.time_from_start.sec = 3  # Move over 3 seconds

        point2 = JointTrajectoryPoint()
        point2.positions = [0.175]  # Second desired positions
        point2.time_from_start.sec = 6  # Move over 6 seconds

        # Add the points to the trajectory message
        traj_msg.points.append(point1)
        traj_msg.points.append(point2)

        # Publish the trajectory
        self.publisher.publish(traj_msg)
        self.get_logger().info("Command sent to move joints:")

        # Stop the timer after sending the command
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = MultiJointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
