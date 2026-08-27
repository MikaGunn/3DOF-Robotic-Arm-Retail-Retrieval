#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from pymoveit2 import MoveIt2


class GoToXYZ(Node):
    def __init__(self):
        super().__init__("go_to_xyz")

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint_1", "joint_2", "joint_3"],
            base_link_name="base_link",
            end_effector_name="claw_support",
            group_name="arm",
        )

    def go(self, x, y, z):
        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z

        # keep orientation simple
        target.pose.orientation.w = 1.0

        self.get_logger().info(f"Going to: x={x}, y={y}, z={z}")
        self.moveit2.move_to_pose(target.pose, target.header.frame_id)
        self.moveit2.wait_until_executed()


def main():
    rclpy.init()
    node = GoToXYZ()

    try:
        while rclpy.ok():
            s = input("Enter x y z (meters): ").strip()
            if not s:
                continue
            x, y, z = map(float, s.split())
            node.go(x, y, z)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

