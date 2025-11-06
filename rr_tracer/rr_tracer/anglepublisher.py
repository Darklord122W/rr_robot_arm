#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class AnglePublisher(Node):
    def __init__(self):
        super().__init__('anglepublisher')

        # 1) Publisher for the arm controller topic
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        # 2) Define your joint names (must match your URDF/controller)
        self.joint_names = ['joint1', 'joint2']

        # 3) Define the target angles (degrees) you want the arm to move through
        # You can add or remove points as you wish
        angle_sequence_deg = [
            (0, 0),
            (90, 45),
            (180, 90),
            (270, 135),
            (360, 180)
        ]

        # 4) Convert to radians and build a trajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        time_from_start = 2.0  # seconds between points
        for deg1, deg2 in angle_sequence_deg:
            point = JointTrajectoryPoint()
            point.positions = [math.radians(deg1), math.radians(deg2)]
            point.time_from_start.sec = int(time_from_start)
            point.time_from_start.nanosec = int((time_from_start - int(time_from_start)) * 1e9)
            trajectory_msg.points.append(point)
            time_from_start += 3.0

        # 5) Publish once and shut down
        self.get_logger().info('Publishing joint trajectory...')
        self.publisher_.publish(trajectory_msg)
        self.get_logger().info('Trajectory published to /arm_controller/joint_trajectory')

        # Optional: exit automatically after publishing
        self.create_timer(1.0, self.shutdown_after_publish)

    def shutdown_after_publish(self):
        self.get_logger().info('Shutting down anglepublisher node.')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = AnglePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
