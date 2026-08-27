#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

class TrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('trajectory_executor')

        self.joint_names = ['joint_1', 'joint_2', 'joint_3']
        self.current_positions = [0.0, 0.0, 0.0]

        self.js_pub = self.create_publisher(
            JointState, '/joint_states', 10)

        self.traj_sub = self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self.trajectory_callback,
            10)

        self.timer = self.create_timer(0.1, self.publish_joint_states)
        self.get_logger().info('Trajectory executor started')

    def trajectory_callback(self, msg):
        if msg.points:
            positions = list(msg.points[-1].positions)
            if len(positions) >= 3:
                self.current_positions = positions[:3]
                self.get_logger().info(
                    f'Executing trajectory: {self.current_positions}')

    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = self.current_positions
        self.js_pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
