###############################################################################
# FILE: cyber_cat/vision_processor_node.py
#
# Detects a coloured marker (pink ball on the Crazyflie) by HSV thresholding.
#
# HSV is used instead of raw RGB ratios so that detection survives the
# brightness swings of indoor lighting: hue stays stable while value does not.
# The conversion is hand-rolled in NumPy to avoid an OpenCV dependency on the
# TurtleBot's laptop image.
###############################################################################
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np


class VisionProcessor(Node):
    """Subscribe to camera images and publish marker detection results."""

    def __init__(self):
        super().__init__('vision_processor')

        # Parameters
        self.declare_parameter('image_topic', '/turtlebot468/oakd/rgb/preview/image_raw')
        self.declare_parameter('min_target_pixels', 25)
        # HSV hue band for the marker (0-180 scale, OpenCV convention).
        # [140, 170] is pink/magenta — chosen because it is absent from the lab
        # background, which green was not. See README 'Marker choice'.
        self.declare_parameter('hue_low', 140)
        self.declare_parameter('hue_high', 170)
        # HSV saturation minimum (0-255) — filters out greyish/white
        self.declare_parameter('sat_min', 50)
        # HSV value minimum (0-255) — filters out very dark pixels
        self.declare_parameter('val_min', 80)

        img_topic = self.get_parameter('image_topic').value
        self.min_target_pixels = int(self.get_parameter('min_target_pixels').value)
        self.hue_low = int(self.get_parameter('hue_low').value)
        self.hue_high = int(self.get_parameter('hue_high').value)
        self.sat_min = int(self.get_parameter('sat_min').value)
        self.val_min = int(self.get_parameter('val_min').value)

        # Publishers
        self.detection_pub = self.create_publisher(String, '/vision/detection', 10)
        self.target_info_pub = self.create_publisher(String, '/vision/target_info', 10)

        self.create_subscription(
            Image,
            img_topic,
            self._on_image,
            qos_profile_sensor_data
        )

        self.last_detection = None

        self.get_logger().info(
            f'VisionProcessor started — listening on {img_topic}'
        )
        self.get_logger().info(
            f'HSV pink range: H=[{self.hue_low},{self.hue_high}] '
            f'S>={self.sat_min} V>={self.val_min} min_pixels={self.min_target_pixels}'
        )

    def _on_image(self, msg: Image) -> None:
        if msg.height == 0 or msg.width == 0:
            self._publish_no_target('empty_image')
            return

        try:
            image = self._reshape_image(msg)
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=5.0)
            self._publish_no_target('reshape_failed')
            return

        encoding = msg.encoding.lower()
        mask = self._target_mask_hsv(image, encoding)

        detection = self._extract_detection(mask, msg.width, msg.height)

        if detection is None:
            self._publish_no_target('insufficient_target_pixels')
            return

        self._publish_target_found(detection)

    def _reshape_image(self, msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        channels_by_encoding = {
            'rgb8': 3,
            'bgr8': 3,
            'rgba8': 4,
            'bgra8': 4,
        }

        channels = channels_by_encoding.get(encoding)
        if channels is None:
            raise ValueError(f'Unsupported image encoding: {msg.encoding}')

        buffer = np.frombuffer(msg.data, dtype=np.uint8)
        expected_size = msg.height * msg.width * channels

        if buffer.size < expected_size:
            raise ValueError(
                f'Image buffer too small for {msg.width}x{msg.height} {msg.encoding}'
            )

        return buffer[:expected_size].reshape((msg.height, msg.width, channels))

    def _rgb_to_hsv(self, r, g, b):
        """Convert RGB arrays (0-255 uint8) to H (0-180), S (0-255), V (0-255).

        Uses the same convention as OpenCV so parameter values are interchangeable.
        """
        r = r.astype(np.float32)
        g = g.astype(np.float32)
        b = b.astype(np.float32)

        maxc = np.maximum(np.maximum(r, g), b)
        minc = np.minimum(np.minimum(r, g), b)
        diff = maxc - minc

        # Value
        v = maxc

        # Saturation
        s = np.zeros_like(v)
        nonzero = maxc > 0
        s[nonzero] = (diff[nonzero] / maxc[nonzero]) * 255.0

        # Hue
        h = np.zeros_like(v)
        mask = diff > 0

        # Red is max
        red_max = mask & (maxc == r)
        h[red_max] = (60.0 * ((g[red_max] - b[red_max]) / diff[red_max])) % 360

        # Green is max
        green_max = mask & (maxc == g)
        h[green_max] = 60.0 * ((b[green_max] - r[green_max]) / diff[green_max]) + 120.0

        # Blue is max
        blue_max = mask & (maxc == b)
        h[blue_max] = 60.0 * ((r[blue_max] - g[blue_max]) / diff[blue_max]) + 240.0

        h[h < 0] += 360.0
        h = h / 2.0  # Scale to 0-180 (OpenCV convention)

        return h, s, v

    def _target_mask_hsv(self, image: np.ndarray, encoding: str) -> np.ndarray:
        """Return a boolean mask of pixels inside the marker's HSV band."""
        if encoding.startswith('bgr'):
            r = image[:, :, 2]
            g = image[:, :, 1]
            b = image[:, :, 0]
        else:
            r = image[:, :, 0]
            g = image[:, :, 1]
            b = image[:, :, 2]

        h, s, v = self._rgb_to_hsv(r, g, b)

        hue_ok = (h >= self.hue_low) & (h <= self.hue_high)
        sat_ok = s >= self.sat_min
        val_ok = v >= self.val_min

        return hue_ok & sat_ok & val_ok

    def _extract_detection(
        self,
        mask: np.ndarray,
        image_width: int,
        image_height: int
    ):
        coords = np.argwhere(mask)

        if coords.shape[0] < self.min_target_pixels:
            return None

        ys = coords[:, 0]
        xs = coords[:, 1]

        min_x = int(xs.min())
        max_x = int(xs.max())
        min_y = int(ys.min())
        max_y = int(ys.max())

        pixel_count = int(coords.shape[0])

        cx = float(xs.mean())
        cy = float(ys.mean())

        bbox_width = max_x - min_x + 1
        bbox_height = max_y - min_y + 1

        norm_cx = cx / float(image_width)
        norm_cy = cy / float(image_height)
        area_ratio = pixel_count / float(image_width * image_height)

        return {
            'pixel_count': pixel_count,
            'cx': cx,
            'cy': cy,
            'norm_cx': norm_cx,
            'norm_cy': norm_cy,
            'min_x': min_x,
            'max_x': max_x,
            'min_y': min_y,
            'max_y': max_y,
            'bbox_width': bbox_width,
            'bbox_height': bbox_height,
            'image_width': image_width,
            'image_height': image_height,
            'area_ratio': area_ratio,
        }

    def _publish_no_target(self, reason: str) -> None:
        det = String()
        det.data = 'no_target'
        self.detection_pub.publish(det)

        info = String()
        info.data = f'status=no_target reason={reason}'
        self.target_info_pub.publish(info)

        if self.last_detection != 'no_target':
            self.get_logger().info(f'Vision: no_target ({reason})')
            self.last_detection = 'no_target'

    def _publish_target_found(self, detection: dict) -> None:
        det = String()
        det.data = 'target_found'
        self.detection_pub.publish(det)

        info = String()
        info.data = (
            'status=target_found '
            f'cx={detection["cx"]:.1f} '
            f'cy={detection["cy"]:.1f} '
            f'norm_cx={detection["norm_cx"]:.3f} '
            f'norm_cy={detection["norm_cy"]:.3f} '
            f'pixel_count={detection["pixel_count"]} '
            f'area_ratio={detection["area_ratio"]:.6f} '
            f'bbox_min_x={detection["min_x"]} '
            f'bbox_max_x={detection["max_x"]} '
            f'bbox_min_y={detection["min_y"]} '
            f'bbox_max_y={detection["max_y"]} '
            f'bbox_width={detection["bbox_width"]} '
            f'bbox_height={detection["bbox_height"]} '
            f'image_width={detection["image_width"]} '
            f'image_height={detection["image_height"]}'
        )
        self.target_info_pub.publish(info)

        if self.last_detection != 'target_found':
            self.get_logger().info(
                f'Vision: target_found '
                f'(pixels={detection["pixel_count"]}, '
                f'cx={detection["cx"]:.1f}, cy={detection["cy"]:.1f})'
            )
            self.last_detection = 'target_found'

    def destroy_node(self):
        self.get_logger().info('Shutting down VisionProcessor')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
