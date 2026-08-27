import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PointStamped
from moveit_msgs.srv import GetPositionIK

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class MoveToXYZ(Node):
    def __init__(self):
        super().__init__("move_to_xyz")

        # ---- ROS params (search pose input) ----
        self.declare_parameter("x", 0.2)
        self.declare_parameter("y", 0.5)
        self.declare_parameter("z", 1.2)
        self.declare_parameter("duration", 2.5)

        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.z = float(self.get_parameter("z").value)
        self.duration = float(self.get_parameter("duration").value)

        # ---- MoveIt settings ----
        self.group_name = "arm"
        self.ik_link_name = "claw_support"
        self.robot_base_frame = "base_link"

        # ---- Publisher to controller ----
        self.traj_topic = "/arm_controller/joint_trajectory"
        self.pub = self.create_publisher(JointTrajectory, self.traj_topic, 10)

        # ---- IK service client ----
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        self.get_logger().info("Waiting for /compute_ik service...")
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/compute_ik not available. Is MoveIt running?")
            raise RuntimeError("/compute_ik service not available")

        self.get_logger().info(
            f"Moving to SEARCH pose x={self.x}, y={self.y}, z={self.z} in frame {self.robot_base_frame}"
        )

        self.timer = self.create_timer(0.5, self.run_once)
        self.sent = False
        
        self.object_sub = self.create_subscription(
            PointStamped,
            '/detected_object_point',
            self.object_callback,
            10
        )

        self.search_reached = False

    def run_once(self):
        if self.sent:
            return
        self.sent = True

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ik_link_name
        req.ik_request.avoid_collisions = False

        pose = PoseStamped()
        pose.header.frame_id = self.robot_base_frame
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.z
        pose.pose.orientation.w = 1.0

        req.ik_request.pose_stamped = pose

        future = self.ik_client.call_async(req)
        future.add_done_callback(self.on_ik_result)

    def on_ik_result(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"IK service call failed: {e}")
            return

        if res.error_code.val != 1:
            self.get_logger().error(f"IK failed. error_code = {res.error_code.val}")
            return

        joint_state = res.solution.joint_state
        names = list(joint_state.name)
        positions = list(joint_state.position)

        wanted = ["joint_1", "joint_2", "joint_3"]
        target = []
        for j in wanted:
            if j not in names:
                self.get_logger().error(f"IK solution missing {j}. Got joints: {names}")
                return
            target.append(float(positions[names.index(j)]))

        self.get_logger().info(f"IK solution (joint_1..3): {target}")

        traj = JointTrajectory()
        traj.joint_names = wanted

        p1 = JointTrajectoryPoint()
        p1.positions = target

        s1 = int(self.duration)
        n1 = int((self.duration - s1) * 1e9)
        p1.time_from_start = Duration(sec=s1, nanosec=n1)

        traj.points = [p1]

        self.pub.publish(traj)
        self.get_logger().info("Moved to SEARCH pose. Waiting for detection...")
        
        self.search_reached = True
        
    def object_callback(self, msg):
        if not self.search_reached:
            return
        self.get_logger().info(
            f"Detected object point received: x={msg.point.x:.3f}, y={msg.point.y:.3f}, z={msg.point.z:.3f}"
        )

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ik_link_name
        req.ik_request.avoid_collisions = False

        pose = PoseStamped()
        pose.header.frame_id = self.robot_base_frame
        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.position.z = msg.point.z
        pose.pose.orientation.w = 1.0

        req.ik_request.pose_stamped = pose

        future = self.ik_client.call_async(req)
        future.add_done_callback(self.on_object_ik_result)
        
    def on_object_ik_result(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"Object IK service call failed: {e}")
            return

        if res.error_code.val != 1:
            self.get_logger().error(f"Object IK failed. error_code = {res.error_code.val}")
            return

        joint_state = res.solution.joint_state
        names = list(joint_state.name)
        positions = list(joint_state.position)

        wanted = ["joint_1", "joint_2", "joint_3"]
        target = []
        for j in wanted:
            if j not in names:
                self.get_logger().error(f"IK solution missing {j}. Got joints: {names}")
                return
            target.append(float(positions[names.index(j)]))

        traj = JointTrajectory()
        traj.joint_names = wanted

        p1 = JointTrajectoryPoint()
        p1.positions = target

        s1 = int(self.duration)
        n1 = int((self.duration - s1) * 1e9)
        p1.time_from_start = Duration(sec=s1, nanosec=n1)

        traj.points = [p1]

        self.pub.publish(traj)
        self.get_logger().info("Moved to detected object point.")


def main():
    rclpy.init()
    node = MoveToXYZ()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
