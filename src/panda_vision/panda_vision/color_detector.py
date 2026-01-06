#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import tf2_ros
import tf_transformations
from rclpy.duration import Duration


class ColorDetector(Node):
    def __init__(self):
        super().__init__("color_detector_node")

        self.bridge = CvBridge()

        # Subscriber
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)

        # Publisher
        self.pub = self.create_publisher(String, "/color_coordinates", 10)

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Camera intrinsic parameters for later use
        # using values from SDF
        # self.camera_info_sub = self.create_subscription(
        #     CameraInfo, "/camera/camera_info", self.camera_info_callback, 10
        # )
        # self.camera_matrix = None
        self.fx = 585.0
        self.fy = 588.0
        self.cx = 320.0
        self.cy = 160.0

        self.get_logger().info("Color Detector Node has started.")

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV in BGR format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Color ranges (tune if needed)
        color_ranges = {
            "R": [(0, 120, 70), (10, 255, 255)],
            "G": [(55, 200, 200), (60, 255, 255)],
            "B": [(90, 200, 200), (126, 255, 255)],
        }

        for color_id, (lower, upper) in color_ranges.items():
            lower = np.array(lower)
            upper = np.array(upper)
            mask = cv2.inRange(hsv, lower, upper)

            # Noise removal
            # kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt) > 1:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cx_pixel = x + w // 2
                    cy_pixel = y + h // 2

                    # Draw bounding box
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.putText(frame, color_id, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                    # Convert pixel to camera frame coordinates
                    Z = 0.1  # Assume a fixed depth of 0.1 meters
                    Y = (cx_pixel - self.cx) * Z / self.fx * -10
                    X = (cy_pixel - self.cy) * Z / self.fy

                    try:
                        # Lookup transform camera_link to panda_link0
                        t = self.tf_buffer.lookup_transform(
                            "panda_link0", "camera_link", rclpy.time.Time(), timeout=Duration(seconds=1.0)
                        )
                        # Convert to numpy transform matrix
                        translation = np.array(
                            [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
                        )
                        rotation = [
                            t.transform.rotation.x,
                            t.transform.rotation.y,
                            t.transform.rotation.z,
                            t.transform.rotation.w,
                        ]

                        # Create transformation matrix
                        T = tf_transformations.quaternion_matrix(rotation)
                        T[:3, 3] = translation

                        # Transform point from camera frame to base frame
                        point_camera = np.array([X, Y, Z, 1.0])
                        point_base = T @ point_camera

                        # Adjust X coordinate for blue and green
                        if color_id == "B":
                            point_base[1] -= 0.0215
                        elif color_id == "G":
                            point_base[1] += 0.01

                        # Publish color id and coordinates in panda_link0 frame
                        msg_str = f"{color_id},{point_base[0]:.3f},{point_base[1]:.3f},{point_base[2]:.3f}"
                        self.pub.publish(String(data=msg_str))
                        self.get_logger().info(f"Published: {msg_str}")
                    except (
                        tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException,
                    ) as e:
                        self.get_logger().error(f"TF lookup failed: {e}")
                    except Exception as e:
                        self.get_logger().error(f"Unexpected error in transformation: {e}")

        # Show image in window
        cv2.namedWindow("Color Detector", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Color Detector", 640, 320)
        cv2.imshow("Color Detector", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
