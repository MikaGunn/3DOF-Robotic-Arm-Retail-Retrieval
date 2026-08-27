import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState


class ArmTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("arm_trajectory_publisher")

        self.topic_name = "/arm_controller/joint_trajectory"
        self.joint_names = ["joint_1", "joint_2", "joint_3"]

        user_args = self._get_user_args()
        self.target_positions = user_args["positions"]

        self.pub = self.create_publisher(JointTrajectory, self.topic_name, 10)

        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )

        self.home_position = [0.0, -0.3, -0.3]

        self.timer = self.create_timer(1.0, self.start_motion_once)
        self.started = False

        self.get_logger().info(f"Publishing to {self.topic_name}")
        self.get_logger().info(f"Target positions: {self.target_positions}")

    def _get_user_args(self):
        self.declare_parameter("target", [0.0, 0.0, 0.0])

        target = self.get_parameter("target").value

        try:
            positions = [float(target[0]), float(target[1]), float(target[2])]
        except Exception:
            self.get_logger().warn("Param 'target' invalid; using [0,0,0].")
            positions = [0.0, 0.0, 0.0]

        return {"positions": positions}

    def joint_state_callback(self, msg: JointState):
        try:
            name_to_pos = dict(zip(msg.name, msg.position))
            self.current_joint_state = [
                name_to_pos["joint_1"],
                name_to_pos["joint_2"],
                name_to_pos["joint_3"]
            ]
        except KeyError:
            pass

    def _dur(self, t: float) -> Duration:
        sec = int(t)
        nanosec = int((t - sec) * 1e9)
        return Duration(sec=sec, nanosec=nanosec)

    def send_trajectory(self, positions, duration_sec):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0, 0.0, 0.0]
        point.accelerations = [0.0, 0.0, 0.0]
        point.time_from_start = self._dur(duration_sec)

        traj.points = [point]
        self.pub.publish(traj)

        self.get_logger().info(f"Sent trajectory to: {positions} in {duration_sec}s")

    def start_motion_once(self):
        if self.started:
            return

        if self.current_joint_state is None:
            self.get_logger().info("Waiting for /joint_states...")
            return

        self.started = True
        self.execute_sequence()

    def execute_sequence(self):
        goal = self.target_positions
        home = self.home_position

        # Start from home assumption. If needed, can use self.current_joint_state instead.
        q0 = home

        # ---------- GO TO TARGET ----------
        # 1) Move joint_3 only
        p1 = [q0[0], q0[1], goal[2]]
        self.send_trajectory(p1, 1.5)
        self.sleep_sec(1.8)

        # 2) Move joint_1 only
        p2 = [goal[0], q0[1], goal[2]]
        self.send_trajectory(p2, 1.5)
        self.sleep_sec(1.8)

        # 3) Move joint_2 only
        p3 = [goal[0], goal[1], goal[2]]
        self.send_trajectory(p3, 1.5)
        self.sleep_sec(1.8)

        # Wait at target
        self.get_logger().info("Waiting at target for 5 seconds...")
        self.sleep_sec(5.0)

        # ---------- RETURN HOME ----------
        # reverse order: joint_2 -> joint_1 -> joint_3

        r1 = [goal[0], home[1], goal[2]]
        self.send_trajectory(r1, 1.5)
        self.sleep_sec(1.8)

        r2 = [home[0], home[1], goal[2]]
        self.send_trajectory(r2, 1.5)
        self.sleep_sec(1.8)

        r3 = [home[0], home[1], home[2]]
        self.send_trajectory(r3, 1.5)
        self.sleep_sec(1.8)

        self.get_logger().info("Sequence complete.")

    def sleep_sec(self, seconds):
        end_time = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    node = ArmTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
