import math
import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PointStamped


class ViewConfirmApproach(Node):
    def __init__(self):
        super().__init__("view_confirm_approach")

        # ------------------------------------------------------------
        # Only these need to be changed from command line
        # ------------------------------------------------------------
        self.declare_parameter("x", 0.16)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.16)
        self.declare_parameter("target_label", "Apple")

        # ------------------------------------------------------------
        # Saved/default settings
        # ------------------------------------------------------------
        self.declare_parameter("confidence_threshold", 0.482)

        self.declare_parameter("image_center_x", 320.0)
        self.declare_parameter("image_center_y", 240.0)

        self.declare_parameter("pixel_deadband_x", 50.0)
        self.declare_parameter("pixel_deadband_y", 45.0)

        # Centre adjustment settings
        self.declare_parameter("joint_1_step_deg", 2.0)
        self.declare_parameter("joint_3_step_deg", 1.0)

        # Approach movement after object is centred
        self.declare_parameter("joint_2_step_deg", -2.0)

        # Number of small approach steps after centring
        self.declare_parameter("max_approach_steps", 2)

        # Movement duration for each step
        self.declare_parameter("duration", 0.7)

        # Final wait and final pose
        self.declare_parameter("final_wait_sec", 10.0)
        self.declare_parameter("final_joint_1", 0.0)
        self.declare_parameter("final_joint_2", 0.0)
        self.declare_parameter("final_joint_3", -1.5708)
        self.declare_parameter("final_duration", 1.0)

        # If wrong object detected, return immediately or wait?
        # 0.1 = almost immediate return
        self.declare_parameter("wrong_object_return_wait_sec", 0.1)

        # ------------------------------------------------------------
        # Read parameters
        # ------------------------------------------------------------
        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.z = float(self.get_parameter("z").value)

        self.target_label = str(self.get_parameter("target_label").value)
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )

        self.image_center_x = float(self.get_parameter("image_center_x").value)
        self.image_center_y = float(self.get_parameter("image_center_y").value)

        self.pixel_deadband_x = float(self.get_parameter("pixel_deadband_x").value)
        self.pixel_deadband_y = float(self.get_parameter("pixel_deadband_y").value)

        self.joint_1_step = math.radians(
            float(self.get_parameter("joint_1_step_deg").value)
        )

        self.joint_3_step = math.radians(
            float(self.get_parameter("joint_3_step_deg").value)
        )

        self.joint_2_step = math.radians(
            float(self.get_parameter("joint_2_step_deg").value)
        )

        self.max_approach_steps = int(
            self.get_parameter("max_approach_steps").value
        )

        self.duration = float(self.get_parameter("duration").value)

        self.final_wait_sec = float(self.get_parameter("final_wait_sec").value)
        self.final_joint_1 = float(self.get_parameter("final_joint_1").value)
        self.final_joint_2 = float(self.get_parameter("final_joint_2").value)
        self.final_joint_3 = float(self.get_parameter("final_joint_3").value)
        self.final_duration = float(self.get_parameter("final_duration").value)

        self.wrong_object_return_wait_sec = float(
            self.get_parameter("wrong_object_return_wait_sec").value
        )

        # ------------------------------------------------------------
        # Publishers and subscribers
        # ------------------------------------------------------------
        self.pub = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10
        )

        self.label_sub = self.create_subscription(
            String,
            "/detected_object_label",
            self.label_callback,
            10
        )

        self.conf_sub = self.create_subscription(
            Float32,
            "/detected_object_confidence",
            self.confidence_callback,
            10
        )

        self.pixel_sub = self.create_subscription(
            PointStamped,
            "/detected_object_pixel",
            self.pixel_callback,
            10
        )

        # ------------------------------------------------------------
        # Detection values
        # ------------------------------------------------------------
        self.latest_label = None
        self.latest_confidence = 0.0
        self.latest_u = None
        self.latest_v = None

        # ------------------------------------------------------------
        # Joint values
        # ------------------------------------------------------------
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

        self.current_q1 = 0.0
        self.current_q2 = 0.0
        self.current_q3 = 0.0

        # ------------------------------------------------------------
        # State machine flags
        # ------------------------------------------------------------
        self.ik_valid = False
        self.target_pose_sent = False
        self.confirmed = False
        self.finished = False
        self.final_pose_sent = False
        self.shutdown_started = False

        self.approach_steps_done = 0

        self.timer = self.create_timer(1.0, self.run_state_machine)

        self.get_logger().info(
            f"Target position: x={self.x}, y={self.y}, z={self.z}"
        )
        self.get_logger().info(
            f"Target item: {self.target_label}"
        )
        self.get_logger().info(
            f"Saved settings: joint_1_step=2.0 deg, "
            f"joint_2_step=-2.0 deg, joint_3_step=1.0 deg, "
            f"max_approach_steps={self.max_approach_steps}, "
            f"duration={self.duration}"
        )

        self.calculate_ik()

    def calculate_ik(self):
        # Arm dimensions
        L1 = 0.180
        L2 = 0.106
        L3 = 0.120

        x = self.x
        y = self.y
        z = self.z

        self.q1 = math.atan2(y, x)

        r = math.sqrt(x ** 2 + y ** 2)
        s = z - L1

        D = (r ** 2 + s ** 2 - L2 ** 2 - L3 ** 2) / (2 * L2 * L3)

        if D > 1.0 or D < -1.0:
            self.get_logger().error(
                f"Target is outside workspace. D={D:.3f}"
            )
            self.ik_valid = False
            return

        # Elbow-up solution
        self.q3 = math.atan2(
            math.sqrt(1 - D ** 2),
            D
        )

        self.q2 = math.atan2(s, r) - math.atan2(
            L3 * math.sin(self.q3),
            L2 + L3 * math.cos(self.q3)
        )

        self.current_q1 = self.q1
        self.current_q2 = self.q2
        self.current_q3 = self.q3

        self.ik_valid = True

        self.get_logger().info(
            f"IK target pose: "
            f"joint_1={math.degrees(self.q1):.1f} deg, "
            f"joint_2={math.degrees(self.q2):.1f} deg, "
            f"joint_3={math.degrees(self.q3):.1f} deg"
        )

    def label_callback(self, msg):
        self.latest_label = msg.data

    def confidence_callback(self, msg):
        self.latest_confidence = float(msg.data)

    def pixel_callback(self, msg):
        self.latest_u = float(msg.point.x)
        self.latest_v = float(msg.point.y)

    def publish_trajectory(self, q1, q2, q3):
        traj = JointTrajectory()
        traj.joint_names = ["joint_1", "joint_2", "joint_3"]

        point = JointTrajectoryPoint()

        # Do not change servo_driver.
        # These signs match your existing servo mapping.
        point.positions = [q1, -q2, -q3]

        seconds = int(self.duration)
        nanoseconds = int((self.duration - seconds) * 1e9)

        point.time_from_start = Duration(
            sec=seconds,
            nanosec=nanoseconds
        )

        traj.points.append(point)
        self.pub.publish(traj)

        self.get_logger().info(
            f"Published input: "
            f"sent_joint_1={math.degrees(q1):.1f} deg, "
            f"sent_joint_2={math.degrees(-q2):.1f} deg, "
            f"sent_joint_3={math.degrees(-q3):.1f} deg"
        )

    def publish_raw_final_pose(self):
        traj = JointTrajectory()
        traj.joint_names = ["joint_1", "joint_2", "joint_3"]

        point = JointTrajectoryPoint()

        # Raw final pose exactly as you requested:
        # positions: [0.0, 0.0, -1.5708]
        point.positions = [
            self.final_joint_1,
            self.final_joint_2,
            self.final_joint_3
        ]

        seconds = int(self.final_duration)
        nanoseconds = int((self.final_duration - seconds) * 1e9)

        point.time_from_start = Duration(
            sec=seconds,
            nanosec=nanoseconds
        )

        traj.points.append(point)
        self.pub.publish(traj)

    def finish_and_wait_for_final_pose(self, reason, wait_time=None):
        if self.finished:
            return

        self.finished = True

        if wait_time is None:
            wait_time = self.final_wait_sec

        self.get_logger().info(reason)
        self.get_logger().info(
            f"Waiting {wait_time} seconds before moving to final pose..."
        )

        self.create_timer(wait_time, self.move_to_final_pose)

    def run_state_machine(self):
        if self.finished:
            return

        if not self.ik_valid:
            return

        # ------------------------------------------------------------
        # Step 1: Move to target pose first
        # ------------------------------------------------------------
        if not self.target_pose_sent:
            self.get_logger().info(
                "Step 1: Moving arm to given target position."
            )

            self.publish_trajectory(
                self.current_q1,
                self.current_q2,
                self.current_q3
            )

            self.target_pose_sent = True
            return

        # ------------------------------------------------------------
        # Step 2: Check detected object
        # ------------------------------------------------------------

        # No object label received yet
        if self.latest_label is None:
            self.get_logger().info(
                f"Waiting for object detection. "
                f"Detected={self.latest_label}, "
                f"confidence={self.latest_confidence:.3f}"
            )
            return

        # Object label received, but confidence is too low
        if self.latest_confidence < self.confidence_threshold:
            self.get_logger().info(
                f"Waiting for confident detection. "
                f"Detected={self.latest_label}, "
                f"confidence={self.latest_confidence:.3f}"
            )
            return

        # Object detected confidently, but it is NOT the target object
        if self.latest_label.lower() != self.target_label.lower():
            self.finish_and_wait_for_final_pose(
                f"Wrong object detected: {self.latest_label}, "
                f"target was {self.target_label}. "
                f"Skipping centring/approach and returning to final pose.",
                wait_time=self.wrong_object_return_wait_sec
            )
            return

        # Correct object detected
        if not self.confirmed:
            self.get_logger().info(
                f"Correct object confirmed: {self.latest_label}, "
                f"confidence={self.latest_confidence:.3f}"
            )
            self.confirmed = True

        # ------------------------------------------------------------
        # Step 3: Need pixel centre for visual centring
        # ------------------------------------------------------------
        if self.latest_u is None or self.latest_v is None:
            self.get_logger().info("Waiting for detected object pixel centre...")
            return

        error_x = self.latest_u - self.image_center_x
        error_y = self.latest_v - self.image_center_y

        self.get_logger().info(
            f"Pixel centre: u={self.latest_u:.1f}, v={self.latest_v:.1f}, "
            f"error_x={error_x:.1f}, error_y={error_y:.1f}"
        )

        # ------------------------------------------------------------
        # Step 4: Centre left/right using joint_1
        # ------------------------------------------------------------
        if abs(error_x) > self.pixel_deadband_x:
            if error_x < 0:
                # Object is left of image centre
                self.current_q1 += self.joint_1_step
                self.get_logger().info("Object is left. Correcting joint_1.")
            else:
                # Object is right of image centre
                self.current_q1 -= self.joint_1_step
                self.get_logger().info("Object is right. Correcting joint_1.")

            self.publish_trajectory(
                self.current_q1,
                self.current_q2,
                self.current_q3
            )
            return

        # ------------------------------------------------------------
        # Step 5: Centre up/down using joint_3
        # ------------------------------------------------------------
        if abs(error_y) > self.pixel_deadband_y:
            if error_y > 0:
                # Object is lower than image centre
                self.current_q3 += self.joint_3_step
                self.get_logger().info("Object is low in image. Correcting joint_3.")
            else:
                # Object is higher than image centre
                self.current_q3 -= self.joint_3_step
                self.get_logger().info("Object is high in image. Correcting joint_3.")

            self.publish_trajectory(
                self.current_q1,
                self.current_q2,
                self.current_q3
            )
            return

        # ------------------------------------------------------------
        # Step 6: Object is centred, now move closer using joint_2
        # ------------------------------------------------------------
        if self.approach_steps_done < self.max_approach_steps:
            self.get_logger().info(
                "Object is centred. Moving one small step toward item."
            )

            self.current_q2 += self.joint_2_step
            self.approach_steps_done += 1

            self.publish_trajectory(
                self.current_q1,
                self.current_q2,
                self.current_q3
            )

            self.get_logger().info(
                f"Approach step {self.approach_steps_done}/"
                f"{self.max_approach_steps}"
            )
            return

        # ------------------------------------------------------------
        # Step 7: Finished approach, wait 10 seconds, then final pose
        # ------------------------------------------------------------
        self.finish_and_wait_for_final_pose(
            "Finished visual servo approach steps: max steps reached."
        )
        return

    def move_to_final_pose(self):
        if self.final_pose_sent:
            return

        self.final_pose_sent = True

        self.publish_raw_final_pose()

        self.get_logger().info(
            f"Moved to final pose: "
            f"joint_1={self.final_joint_1:.4f}, "
            f"joint_2={self.final_joint_2:.4f}, "
            f"joint_3={self.final_joint_3:.4f}"
        )

        self.create_timer(self.final_duration + 1.0, self.shutdown)

    def shutdown(self):
        if self.shutdown_started:
            return

        self.shutdown_started = True
        self.get_logger().info("Done. Shutting down node.")

        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ViewConfirmApproach()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

        node.destroy_node()


if __name__ == "__main__":
    main()
