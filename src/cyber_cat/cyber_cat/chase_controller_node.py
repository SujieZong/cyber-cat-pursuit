###############################################################################
# FILE: cyber_cat/chase_controller_node.py
#
# Proportional visual servo. Turns the marker's position in the image into a
# Twist, with no metric depth anywhere in the loop.
#
# Two ideas carry the controller:
#   1. Bearing from pixels. The marker's normalised horizontal position
#      (norm_cx) is the whole steering error; forward speed is throttled when
#      the turn is hard so the robot does not arc away from the target.
#   2. Range from apparent size. area_ratio (fraction of frame that is marker)
#      stands in for distance. Forward speed ramps to zero across a brake band,
#      which gives a standoff without a depth sensor.
#
# Detection flicker is absorbed by a short grace period that turns toward the
# last known bearing at zero forward speed. An earlier version coasted forward
# during grace and drove the robot blind into furniture.
###############################################################################
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class ChaseController(Node):
    """Drives the TurtleBot toward a visually tracked marker."""

    def __init__(self):
        super().__init__('chase_controller')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('cmd_vel_topic', '/turtlebot468/cmd_vel')
        self.declare_parameter('max_linear_speed', 0.4)      # m/s
        self.declare_parameter('max_angular_speed', 1.2)     # rad/s
        self.declare_parameter('angular_gain', 1.5)
        self.declare_parameter('linear_gain', 0.35)
        self.declare_parameter('centering_deadzone', 0.05)   # normalised
        self.declare_parameter('search_turn_speed', 0.5)     # rad/s
        self.declare_parameter('target_grace_sec', 1.0)

        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.max_v = float(self.get_parameter('max_linear_speed').value)
        self.max_w = float(self.get_parameter('max_angular_speed').value)
        self.k_angular = float(self.get_parameter('angular_gain').value)
        self.k_linear = float(self.get_parameter('linear_gain').value)
        self.deadzone = float(self.get_parameter('centering_deadzone').value)
        self.search_w = float(self.get_parameter('search_turn_speed').value)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        self.get_logger().info(f'Publishing cmd_vel to: {cmd_topic}')

        # Subscribers
        self.create_subscription(String, '/mission/state', self._on_state, 10)
        self.create_subscription(
            String, '/vision/target_info', self._on_target_info, 10
        )

        # Internal state
        self.state = 'IDLE'

        # Vision tracking variables
        self.target_visible = False
        self.norm_cx = 0.5
        self.area_ratio = 0.0

        # Grace period: keep tracking the last known bearing for this long
        # after losing sight, instead of immediately spinning to re-acquire.
        self.target_grace_sec = float(
            self.get_parameter('target_grace_sec').value
        )
        self.last_target_time = 0.0
        self.last_norm_cx = 0.5
        self.last_area_ratio = 0.0

        # Standoff using vision area_ratio (larger = closer). Forward motion is
        # ramped down between brake_start and standoff, then held at zero.
        self.declare_parameter('standoff_forward_area_ratio', 0.018)
        self.declare_parameter('approach_brake_area_ratio', 0.007)
        self.standoff_area = float(
            self.get_parameter('standoff_forward_area_ratio').value
        )
        self.brake_start_area = float(
            self.get_parameter('approach_brake_area_ratio').value
        )
        if self.brake_start_area >= self.standoff_area:
            self.get_logger().warning(
                'approach_brake_area_ratio >= standoff_forward_area_ratio; '
                'using brake_start = 0.5 * standoff'
            )
            self.brake_start_area = 0.5 * self.standoff_area

        self.get_logger().info(
            f'Standoff: brake from area>={self.brake_start_area:.4f}, '
            f'no forward when area>={self.standoff_area:.4f}'
        )

        # Control loop timer
        self.timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info('ChaseController started (vision-based servo)')

    def _on_state(self, msg):
        """Receive mission state updates."""
        self.state = msg.data

    def _on_target_info(self, msg):
        """Parse vision output and extract target position information."""
        data = msg.data

        if 'status=target_found' in data:
            self.target_visible = True
            self.last_target_time = time.monotonic()

            parts = data.split()

            for p in parts:
                if p.startswith('norm_cx='):
                    self.norm_cx = float(p.split('=')[1])
                    self.last_norm_cx = self.norm_cx
                elif p.startswith('area_ratio='):
                    self.area_ratio = float(p.split('=')[1])
                    self.last_area_ratio = self.area_ratio
        else:
            self.target_visible = False

    def _target_recently_seen(self):
        """Return True if target was seen within the grace period."""
        return (time.monotonic() - self.last_target_time) < self.target_grace_sec

    def _control_loop(self):
        """Main control loop for visual tracking."""
        cmd = Twist()

        # IDLE and CAPTURE: full stop
        if self.state in ('IDLE', 'CAPTURE'):
            self.cmd_pub.publish(cmd)
            return

        # SEARCH behavior: rotate in place to look for target
        if self.state == 'SEARCH':
            cmd.angular.z = self.search_w
            cmd.linear.x = 0.0
            self.cmd_pub.publish(cmd)
            return

        # ── CHASE behavior ────────────────────────────────────────────
        # Visible: full visual servo (forward + turn).
        if self.target_visible:
            use_cx = self.norm_cx
            use_area = self.area_ratio
        elif self._target_recently_seen():
            # Grace: ride through flicker without forward coast (that was
            # causing blind driving). Turn toward last bearing or scan in place.
            error = self.last_norm_cx - 0.5
            if abs(error) < self.deadzone:
                # Last seen dead ahead but now gone — scan rather than drive.
                cmd.angular.z = self.search_w
            else:
                cmd.angular.z = max(
                    -self.max_w,
                    min(self.max_w, -self.k_angular * error),
                )
            cmd.linear.x = 0.0
            self.cmd_pub.publish(cmd)
            return
        else:
            # Grace expired — spin to re-acquire
            cmd.angular.z = self.search_w
            cmd.linear.x = 0.0
            self.cmd_pub.publish(cmd)
            return

        # Compute horizontal error from image center (target visible)
        error = use_cx - 0.5

        # Deadzone to reduce jitter around centre
        if abs(error) < self.deadzone:
            error = 0.0

        # Angular velocity: proportional to horizontal image error
        cmd.angular.z = max(
            -self.max_w,
            min(self.max_w, -self.k_angular * error)
        )

        # Forward velocity — scale with how centered the target is
        # Move faster when target is centered, slower when turning hard
        center_factor = 1.0 - min(1.0, abs(error) * 2.0)
        cmd.linear.x = min(
            self.max_v, self.k_linear * (0.4 + 0.6 * center_factor)
        )

        # Distance / standoff: scale down forward speed as target grows in frame
        if use_area >= self.standoff_area:
            cmd.linear.x = 0.0
        elif use_area > self.brake_start_area:
            span = self.standoff_area - self.brake_start_area
            t = (self.standoff_area - use_area) / span
            cmd.linear.x *= max(0.0, min(1.0, t))

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    node = ChaseController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
