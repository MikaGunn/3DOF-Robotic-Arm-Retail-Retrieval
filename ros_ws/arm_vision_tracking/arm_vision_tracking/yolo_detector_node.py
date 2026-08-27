#!/usr/bin/env python3

import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Vector3Stamped
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point, do_transform_vector3
from ultralytics import YOLO

CLASS_NAMES = {0: 'Apple', 1: 'Banana', 2: 'Lemon', 3: 'Orange'}
CLASS_COLOURS = {
    0: (0, 200, 0),
    1: (0, 215, 255),
    2: (0, 255, 200),
    3: (0, 140, 255),
}

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter('model_path',
            os.environ.get('MODEL_PATH', '/ros_ws/models/V3weights.pt'))
        self.declare_parameter('confidence_threshold', 0.482)

        model_path = self.get_parameter('model_path').value
        self.conf = self.get_parameter('confidence_threshold').value

        self.get_logger().info(f'Loading YOLO model from {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('Model loaded successfully')

        self.bridge = CvBridge()

        self.fx = 500.0
        self.fy = 500.0
        self.cx = 320.0
        self.cy = 240.0
        self.camera_frame = 'camera_frame'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame = 'base_link'
        self.table_z = 0.85

        self.create_subscription(CameraInfo, '/camera/camera_info',
            self.camera_info_callback, 10)
        self.create_subscription(Image, '/camera/image_raw',
            self.image_callback, 10)

        self.point_pub = self.create_publisher(
            PointStamped, '/detected_object_point', 10)
        self.pixel_pub = self.create_publisher(
            PointStamped, '/detected_object_pixel', 10)
        self.image_pub = self.create_publisher(
            Image, '/camera/image_annotated', 10)
        self.label_pub = self.create_publisher(
            String, '/detected_object_label', 10)
        self.confidence_pub = self.create_publisher(
            Float32, '/detected_object_confidence', 10)

        self.get_logger().info('YOLO detector node started')

    def camera_info_callback(self, msg):
        if msg.k[0] != 0.0:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_frame = msg.header.frame_id

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        results = self.model.predict(source=frame, conf=self.conf, verbose=False)
        boxes = results[0].boxes if results and results[0].boxes else []

        best_box = None
        best_conf = 0.0

        for box in boxes:
            conf = float(box.conf[0].item())
            if conf > best_conf:
                best_conf = conf
                best_box = box

        annotated = frame.copy()

        if best_box is not None:
            cls_id = int(best_box.cls[0].item())
            x1, y1, x2, y2 = [int(v) for v in best_box.xyxy[0].tolist()]
            u = (x1 + x2) // 2
            v = (y1 + y2) // 2

            colour = CLASS_COLOURS.get(cls_id, (255, 255, 255))
            label = f'{CLASS_NAMES.get(cls_id, "?")} {best_conf:.2f}'

            cv2.rectangle(annotated, (x1, y1), (x2, y2), colour, 2)
            cv2.putText(annotated, label, (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
            cv2.circle(annotated, (u, v), 5, colour, -1)

            self.get_logger().info(
                f'Detected {CLASS_NAMES.get(cls_id, "?")} '
                f'conf={best_conf:.3f} pixel=({u},{v})')

            label_msg = String()
            label_msg.data = CLASS_NAMES.get(cls_id, 'Unknown')
            self.label_pub.publish(label_msg)

            confidence_msg = Float32()
            confidence_msg.data = float(best_conf)
            self.confidence_pub.publish(confidence_msg)

            pixel_msg = PointStamped()
            pixel_msg.header = msg.header
            pixel_msg.point.x = float(u)
            pixel_msg.point.y = float(v)
            pixel_msg.point.z = 0.0
            self.pixel_pub.publish(pixel_msg)

            try:
                point = self.pixel_to_plane(u, v, msg.header.stamp)
                if point is not None:
                    self.point_pub.publish(point)
            except Exception as e:
                self.get_logger().warn(f'TF not ready: {e}')

        try:
            ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            ann_msg.header = msg.header
            self.image_pub.publish(ann_msg)
        except Exception as e:
            self.get_logger().error(f'Image publish error: {e}')

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
            self.base_frame, self.camera_frame, rclpy.time.Time())

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

        point = PointStamped()
        point.header.frame_id = self.base_frame
        point.header.stamp = stamp
        point.point.x = origin_base.point.x + t * dx
        point.point.y = origin_base.point.y + t * dy
        point.point.z = self.table_z

        return point


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
