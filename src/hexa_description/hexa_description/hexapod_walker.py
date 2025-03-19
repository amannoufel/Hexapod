import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class HexapodGaitController(Node):
    def __init__(self):
        super().__init__('hexapod_gait_controller')
        
        self.joint_names = [
            "Revolute 1", "Revolute 2", "Revolute 3",
            "Revolute 4", "Revolute 5", "Revolute 6",
            "Revolute 7", "Revolute 8", "Revolute 9",
            "Revolute 10", "Revolute 11", "Revolute 12",
            "Revolute 13", "Revolute 14", "Revolute 15",
            "Revolute 16", "Revolute 17", "Revolute 18"
        ]

        # Define left/right legs based on URDF
        self.left_legs = ["C1_1", "C3_1", "C5_1"]
        self.right_legs = ["C2_1", "C4_1", "C6_1"]

        # Tripod groups (front-left, middle-right, back-left)
        self.tripod1 = [0, 1, 2, 9, 10, 11, 15, 16, 17]  # Legs 1, 4, 6
        self.tripod2 = [3, 4, 5, 6, 7, 8, 12, 13, 14]     # Legs 2, 3, 5

        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
        )
        self.current_positions = [0.0] * 18
        self.timer = self.create_timer(2.0, self.execute_gait_cycle)
        self.phase = 0

    def create_trajectory_point(self, positions, time_sec):
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = time_sec
        return point

    def execute_gait_cycle(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        target_positions = self.current_positions.copy()

        if self.phase % 2 == 0:
            # Move tripod1 (lift and swing forward)
            for idx in self.tripod1:
                # Determine leg side
                is_left = self.joint_names[idx] in self.left_legs
                if idx % 3 == 0:  # Coxa joint
                    if is_left:
                        target_positions[idx] = math.radians(-20)  # Left legs rotate clockwise
                    else:
                        target_positions[idx] = math.radians(20)    # Right legs rotate counter-clockwise
                elif idx % 3 == 1:  # Femur joint (lift)
                    target_positions[idx] = math.radians(10)        # Within URDF limit
                elif idx % 3 == 2:  # Tibia joint (extend)
                    target_positions[idx] = math.radians(-30)       # Within URDF limit

            # Stance phase for tripod2 (keep legs planted)
            for idx in self.tripod2:
                target_positions[idx] = 0.0

        else:
            # Reverse tripod groups
            for idx in self.tripod2:
                # Determine leg side
                is_left = self.joint_names[idx] in self.left_legs
                if idx % 3 == 0:  # Coxa joint
                    if is_left:
                        target_positions[idx] = math.radians(-20)
                    else:
                        target_positions[idx] = math.radians(20)
                elif idx % 3 == 1:  # Femur joint
                    target_positions[idx] = math.radians(10)
                elif idx % 3 == 2:  # Tibia joint
                    target_positions[idx] = math.radians(-30)

            # Stance phase for tripod1
            for idx in self.tripod1:
                target_positions[idx] = 0.0

        trajectory.points.append(self.create_trajectory_point(target_positions, 2))
        self.current_positions = target_positions
        self.trajectory_pub.publish(trajectory)
        self.get_logger().info(f"Phase {self.phase}: Published gait trajectory")
        self.phase += 1

def main(args=None):
    rclpy.init(args=args)
    controller = HexapodGaitController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()