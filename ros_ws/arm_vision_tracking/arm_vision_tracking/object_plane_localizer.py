#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

from geometry_msgs.msg import PointStamped, Vector3Stamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point, do_transform_vector3


class ObjectPlaneLocalizer(Node):
    def __init__(self):
        super().__init__('object_plane_localizer')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_frame = None

        self.pixel_pub = self.create_publisher(
            PointStamped,
            '/detected_object_pixel',
            10
        )

        self.point_pub = self.create_publisher(
            PointStamped,
            '/detected_object_point',
            10
        )

        self.min_contour_area = 300

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.base_frame = 'base_link'
        self.table_z = 0.85

        self.get_logger().info('Object plane localizer started.')

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_frame = msg.header.frame_id

    def image_callback(self, msg):
        if self.fx is None:
            self.get_logger().warn('Waiting for camera info...')
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        center = self.detect_green_object(frame)

        if center is not None:
            u, v = center

            pixel_msg = PointStamped()
            pixel_msg.header = msg.header
            pixel_msg.point.x = float(u)
            pixel_msg.point.y = float(v)
            pixel_msg.point.z = 0.0
            self.pixel_pub.publish(pixel_msg)

            try:
                point_base = self.pixel_to_plane(u, v, msg.header.stamp)
                if point_base is not None:
                    self.point_pub.publish(point_base)
                    self.get_logger().info(
                        f'Object at base frame: x={point_base.point.x:.3f}, '
                        f'y={point_base.point.y:.3f}, z={point_base.point.z:.3f}'
                    )
            except Exception as e:
                self.get_logger().warn(f'TF not ready yet: {e}')
                return

    def pixel_to_plane(self, u, v, stamp):
        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy

        origin_cam = PointStamped()
        origin_cam.header.frame_id = self.camera_frame
        origin_cam.header.stamp = stamp
        origin_cam.point.x = 0.0
        origin_cam.point.y = 0.0
        origin_cam.point.z = 0.0

        ray_cam = Vector3Stamped()
        ray_cam.header.frame_id = self.camera_frame
        ray_cam.header.stamp = stamp
        ray_cam.vector.x = x
        ray_cam.vector.y = y
        ray_cam.vector.z = 1.0

        transform = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.camera_frame,
            rclpy.time.Time()
        )

        origin_base = do_transform_point(origin_cam, transform)
        ray_base = do_transform_vector3(ray_cam, transform)

        dx = ray_base.vector.x
        dy = ray_base.vector.y
        dz = ray_base.vector.z

        if abs(dz) < 1e-6:
            return None

        t = (self.table_z - origin_base.point.z) / dz

        if t < 0:
            return None

        point_base = PointStamped()
        point_base.header.frame_id = self.base_frame
        point_base.header.stamp = stamp
        point_base.point.x = origin_base.point.x + t * dx
        point_base.point.y = origin_base.point.y + t * dy
        point_base.point.z = self.table_z

        return point_base
    
    def detect_green_object(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower = np.array([0, 50, 50])
        upper = np.array([180, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < self.min_contour_area:
            return None

        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None

        u = int(M['m10'] / M['m00'])
        v = int(M['m01'] / M['m00'])

        return (u, v)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPlaneLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()