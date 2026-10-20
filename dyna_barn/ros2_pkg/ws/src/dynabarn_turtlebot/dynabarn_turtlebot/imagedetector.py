#!/usr/bin/env python3
# imagedetector: subscribes to RGB + DEPTH, detects red regions,
# draws boxes with depth, and publishes:
#   - annotated image on /imagedetector/image_annotated
#   - closest barn distance (meters) on /barn_distance

import math
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


def qos_sensor(depth=10) -> QoSProfile:
    # BEST_EFFORT matches typical sensor publishers (camera/depth)
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT
    )


class imagedetector(Node):
    def __init__(self):
        super().__init__('imagedetector')

        # --- Parameters ---
        self.declare_parameter('input_image_topic', '/turtlebot3/rgb_cam/image_raw')
        self.declare_parameter('depth_image_topic', '/turtlebot3/depth_cam/image_raw')  # <- your depth topic
        self.declare_parameter('output_image_topic', '/imagedetector/image_annotated')

        self.declare_parameter('min_area_px', 1200)
        self.declare_parameter('blur_ksize', 5)
        self.declare_parameter('dilate_iter', 1)
        self.declare_parameter('erode_iter', 0)
        self.declare_parameter('depth_median_win', 5)       # median window (pixels) around box center
        self.declare_parameter('publish_when_missing', True) # heartbeat (NaN) when no barn

        # --- QoS and pubs/subs ---
        qos_sub = qos_sensor()
        qos_img_pub = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.bridge = CvBridge()

        self.sub_img = self.create_subscription(
            Image,
            self.get_parameter('input_image_topic').value,
            self.on_image,
            qos_sub
        )
        self.sub_depth = self.create_subscription(
            Image,
            self.get_parameter('depth_image_topic').value,
            self.on_depth,
            qos_sub
        )

        self.pub_img = self.create_publisher(
            Image,
            self.get_parameter('output_image_topic').value,
            qos_img_pub
        )
        self.pub_closest = self.create_publisher(Float32, '/barn_distance', 10)

        # Depth cache
        self.latest_depth = None           # numpy array (H x W), meters or mm depending on encoding
        self.latest_depth_encoding = None  # e.g., "32FC1" or "16UC1"

        self.get_logger().info("imagedetector running (RGB + Depth fusion)")

    # ---- Depth callback ----
    def on_depth(self, msg: Image):
        self.latest_depth_encoding = msg.encoding
        try:
            # 'passthrough' keeps original encoding: 32FC1 (m) or 16UC1 (mm)
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.get_logger().info(
                f"Depth image received: {self.latest_depth.shape} ({msg.encoding})", once=True
            )
        except Exception as e:
            self.get_logger().warn(f"cv_bridge depth fail: {e}")
            self.latest_depth = None

    # ---- Image callback ----
    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        # Preprocess
        k = int(self.get_parameter('blur_ksize').value)
        if k % 2 == 0:
            k += 1
        frame_blur = cv2.GaussianBlur(frame, (k, k), 0) if k >= 3 else frame

        # HSV red mask
        hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 90, 90), (10, 255, 255)) | \
               cv2.inRange(hsv, (170, 90, 90), (180, 255, 255))

        d_iter = int(self.get_parameter('dilate_iter').value)
        e_iter = int(self.get_parameter('erode_iter').value)
        if d_iter > 0:
            mask = cv2.dilate(mask, None, iterations=d_iter)
        if e_iter > 0:
            mask = cv2.erode(mask, None, iterations=e_iter)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        annotated = frame.copy()
        min_area = int(self.get_parameter('min_area_px').value)

        # Track closest
        closest_dist = None
        closest_box = None

        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            if w * h < min_area:
                continue

            # Draw base box
            cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Depth at the box center (median over a small window)
            dist_txt = "n/a"
            if self.latest_depth is not None:
                u_center = x + w / 2.0
                v_center = y + h / 2.0
                rng = self._depth_at(u_center, v_center, win=int(self.get_parameter('depth_median_win').value))
                if rng is not None and math.isfinite(rng) and rng > 0.0:
                    dist_txt = f"{rng:.2f} m"
                    if (closest_dist is None) or (rng < closest_dist):
                        closest_dist = rng
                        closest_box = (x, y, w, h)

            cv2.putText(
                annotated, f"red_barn {dist_txt}", (x, max(0, y - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA
            )

        # Highlight & label closest
        if closest_dist is not None and closest_box is not None:
            x, y, w, h = closest_box
            cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 255), 3)
            cv2.putText(
                annotated, f"closest {closest_dist:.2f} m",
                (x, max(0, y - 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA
            )

        # Publish annotated image
        out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out.header = msg.header
        self.pub_img.publish(out)

        # Publish closest distance (or heartbeat)
        if closest_dist is not None:
            self.pub_closest.publish(Float32(data=float(closest_dist)))
        elif bool(self.get_parameter('publish_when_missing').value):
            self.pub_closest.publish(Float32(data=float('nan')))

    # ---- Helpers ----
    def _depth_at(self, u, v, win=5):
        """
        Return depth (meters) around pixel (u,v) using median in a win x win window.
        Supports depth encodings 32FC1 (meters) and 16UC1 (millimeters).
        """
        dimg = self.latest_depth
        if dimg is None:
            return None

        h, w = dimg.shape[:2]
        u = int(round(u)); v = int(round(v))
        if u < 0 or v < 0 or u >= w or v >= h:
            return None

        half = max(1, win // 2)
        x0, x1 = max(0, u - half), min(w - 1, u + half)
        y0, y1 = max(0, v - half), min(h - 1, v + half)
        roi = dimg[y0:y1+1, x0:x1+1]

        enc = (self.latest_depth_encoding or '').upper()
        if '16U' in enc:
            vals = roi.astype(np.float32) / 1000.0  # mm -> m
        else:
            # assume meters (32FC1 or anything else)
            vals = roi.astype(np.float32)

        # Remove invalids (<=0 or NaN/Inf)
        vals = vals[np.isfinite(vals)]
        vals = vals[vals > 0.0]
        if vals.size == 0:
            return None
        return float(np.median(vals))


def main():
    rclpy.init()
    node = imagedetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()