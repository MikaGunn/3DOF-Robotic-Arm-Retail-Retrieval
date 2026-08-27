#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class VisualServoNode(Node):
    def __init__(self):
        super().__init__('visual_servo_node')

        self.frame_w = 640.0
        self.frame_h = 480.0

        self.joint_1_min = -1.4
        self.joint_1_max = 1.4
        self.joint_2_min = -0.3
        self.joint_2_max = 0.5
        self.joint_3_fixed = -0.3

        self.pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10)

        self.create_subscription(
            PointStamped,
            '/detected_object_pixel',
            self.pixel_callback,
            10)

        self.get_logger().info('Visual servo node started')

    def pixel_callback(self, msg):
        u = msg.point.x
        v = msg.point.y

        joint_1 = self.map_value(u, 0, self.frame_w,
                                  self.joint_1_max, self.joint_1_min)

        joint_2 = self.map_value(v, 0, self.frame_h,
                                  self.joint_2_max, self.joint_2_min)

        joint_3 = self.joint_3_fixed

        traj = JointTrajectory()
        traj.joint_names = ['joint_1', 'joint_2', 'joint_3']

        point = JointTrajectoryPoint()
        point.positions = [joint_1, joint_2, joint_3]
        point.time_from_start = Duration(sec=0, nanosec=100000000)

        traj.points = [point]
        self.pub.publish(traj)

        self.get_logger().info(
            f'Pixel ({u:.0f},{v:.0f}) -> joints ({joint_1:.3f}, {joint_2:.3f}, {joint_3:.3f})')

    def map_value(self, value, in_min, in_max, out_min, out_max):
        return out_min + (float(value - in_min) / float(in_max - in_min)) * (out_max - out_min)

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
