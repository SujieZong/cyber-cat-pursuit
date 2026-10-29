###############################################################################
# FILE: cyber_cat/mission_executive_node.py
#
# Mission state machine: IDLE → SEARCH → CHASE → CAPTURE.
#
# The node owns every transition so the chase controller can stay a pure
# reflex loop. Two guards keep the machine from thrashing on real hardware,
# where single-frame detection is noisy:
#
#   * Rolling window. A deque of the last N detection results; entering CHASE
#     needs a confirmed majority, declaring loss needs a confirmed minority.
#     Hysteresis, not a single-frame edge.
#   * Capture requires both. A large area_ratio AND a confirmed detection,
#     sustained. Area alone spikes when the robot yaws past clutter.
#
# Mission state also drives the Create 3 light ring, which turns out to be the
# fastest way to debug a robot you are chasing around a lab on foot.
###############################################################################
import collections
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# TurtleBot 4 Create 3 light ring. Optional: the state machine runs headless
# in simulation, where irobot_create_msgs is not installed.
try:
    from irobot_create_msgs.msg import LedColor, LightringLeds
    LIGHTRING_AVAILABLE = True
except ImportError:
    LIGHTRING_AVAILABLE = False


class MissionExecutive(Node):
    """Top-level state machine: IDLE → SEARCH → CHASE → CAPTURE."""

    def __init__(self):
        super().__init__('mission_executive')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('mission_rate', 10.0)
        self.declare_parameter('search_timeout', 90.0)
        self.declare_parameter('detection_window_size', 10)
        self.declare_parameter('detection_confirm_threshold', 7)
        self.declare_parameter('detection_loss_threshold', 2)
        self.declare_parameter('target_lost_timeout', 5.0)
        self.declare_parameter('capture_area_threshold', 0.01)
        self.declare_parameter('capture_confirm_time', 2.0)
        self.declare_parameter('celebrate_timeout', 5.0)
        self.declare_parameter('lightring_topic', '/turtlebot468/cmd_lightring')

        rate = self.get_parameter('mission_rate').value
        self.search_timeout = self.get_parameter('search_timeout').value
        window_size = int(self.get_parameter('detection_window_size').value)
        self.confirm_thresh = int(
            self.get_parameter('detection_confirm_threshold').value
        )
        self.loss_thresh = int(
            self.get_parameter('detection_loss_threshold').value
        )
        self.target_lost_timeout = self.get_parameter('target_lost_timeout').value
        self.capture_area_threshold = self.get_parameter('capture_area_threshold').value
        self.capture_confirm_time = self.get_parameter('capture_confirm_time').value
        self.celebrate_timeout = self.get_parameter('celebrate_timeout').value

        # ── State ─────────────────────────────────────────────────────
        self.state = 'IDLE'
        self.state_entered_at = time.monotonic()

        # Detection flicker filter (rolling window)
        self.detection_history = collections.deque(
            [0] * window_size, maxlen=window_size
        )
        self.last_confirmed_detection_time = 0.0

        # Capture sustain timer
        self.capture_timer_start = None

        # Area ratio from vision (for capture detection)
        self.area_ratio = 0.0

        # ── Publishers ────────────────────────────────────────────────
        self.state_pub = self.create_publisher(String, '/mission/state', 10)

        if LIGHTRING_AVAILABLE:
            lightring_topic = self.get_parameter('lightring_topic').value
            self.light_pub = self.create_publisher(
                LightringLeds, lightring_topic, 10
            )
            self.get_logger().info(
                f'Light ring control enabled on {lightring_topic}'
            )
        else:
            self.light_pub = None
            self.get_logger().warning(
                'irobot_create_msgs not found — light ring disabled'
            )

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(
            String, '/vision/detection', self._on_detection, 10
        )
        self.create_subscription(
            String, '/vision/target_info', self._on_target_info, 10
        )
        self.create_subscription(
            String, '/voice/command', self._on_voice, 10
        )

        # ── Main loop ─────────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info('MissionExecutive started — state: IDLE')
        self._set_light()

    # ==================================================================
    # State transitions
    # ==================================================================

    def _set_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f'State: {self.state} → {new_state}')
            self.state = new_state
            self.state_entered_at = time.monotonic()
            # Reset capture timer on any state change
            self.capture_timer_start = None
            self._set_light()

    def _time_in_state(self):
        return time.monotonic() - self.state_entered_at

    def _confirmed_detection(self):
        """True when enough recent frames have target_found."""
        return sum(self.detection_history) >= self.confirm_thresh

    def _confirmed_loss(self):
        """True when few recent frames have target_found."""
        return sum(self.detection_history) < self.loss_thresh

    # ==================================================================
    # Callbacks
    # ==================================================================

    def _on_detection(self, msg):
        """Update rolling detection window."""
        if msg.data == 'target_found':
            self.detection_history.append(1)
            self.last_confirmed_detection_time = time.monotonic()
        else:
            self.detection_history.append(0)

    def _on_target_info(self, msg):
        """Parse area_ratio from vision for capture detection."""
        for part in msg.data.split():
            if part.startswith('area_ratio='):
                self.area_ratio = float(part.split('=')[1])
                break

    def _on_voice(self, msg):
        """Handle operator commands (see voice_command_node)."""
        cmd = msg.data.strip().lower()
        if 'start' in cmd and self.state == 'IDLE':
            self._set_state('SEARCH')
            self.get_logger().info('Command: start → SEARCH')
        elif 'stop' in cmd or 'abort' in cmd:
            self._set_state('IDLE')
            self.get_logger().info('Command: stop → IDLE')

    # ==================================================================
    # Main tick (10 Hz)
    # ==================================================================

    def _tick(self):
        now = time.monotonic()

        # ── IDLE: do nothing, wait for start ──────────────────────────
        if self.state == 'IDLE':
            pass

        # ── SEARCH: look for confirmed detection or timeout ───────────
        elif self.state == 'SEARCH':
            if self._confirmed_detection():
                self._set_state('CHASE')
                self.get_logger().info('Confirmed detection → CHASE')
            elif self._time_in_state() > self.search_timeout:
                self._set_state('IDLE')
                self.get_logger().info('Search timeout → IDLE')

        # ── CHASE: pursue target, check capture or loss ───────────────
        elif self.state == 'CHASE':
            # Capture needs a large marker AND a confirmed detection. Area
            # alone spikes for a single frame when the robot yaws past
            # background clutter, or when the drone crosses close to the lens
            # on its way elsewhere — neither is a capture.
            if (self.area_ratio > self.capture_area_threshold
                    and self._confirmed_detection()):
                if self.capture_timer_start is None:
                    self.capture_timer_start = now
                elif (now - self.capture_timer_start) >= self.capture_confirm_time:
                    self._set_state('CAPTURE')
                    self.get_logger().info('Target captured!')
            else:
                # Either condition failing resets the sustain timer.
                self.capture_timer_start = None

            # Check target loss
            if self.state == 'CHASE':  # still in CHASE (not just captured)
                time_since_last = now - self.last_confirmed_detection_time
                if (self._confirmed_loss()
                        and time_since_last > self.target_lost_timeout):
                    self._set_state('SEARCH')
                    self.get_logger().info(
                        f'Target lost for {time_since_last:.1f}s → SEARCH'
                    )

        # ── CAPTURE: celebrate then reset ─────────────────────────────
        elif self.state == 'CAPTURE':
            if self._time_in_state() > self.celebrate_timeout:
                self._set_state('IDLE')
                self.get_logger().info('Celebrate done → IDLE')

        # ── Publish current state ─────────────────────────────────────
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

        # Update light (for flashing/pulsing effects)
        self._set_light()

    # ==================================================================
    # Light ring control
    # ==================================================================

    def _set_light(self):
        if self.light_pub is None:
            return

        msg = LightringLeds()
        msg.override_system = True

        if self.state == 'IDLE':
            # White dim — standby
            color = self._make_color(80, 80, 80)
            msg.leds = [color] * 6

        elif self.state == 'SEARCH':
            # Blue — scanning
            color = self._make_color(0, 0, 255)
            msg.leds = [color] * 6

        elif self.state == 'CHASE':
            # Yellow — pursuing
            color = self._make_color(255, 200, 0)
            msg.leds = [color] * 6

        elif self.state == 'CAPTURE':
            # Green flashing — victory
            tick = int(self._time_in_state() * 4) % 2  # 4 Hz flash
            if tick == 0:
                color = self._make_color(0, 255, 0)
            else:
                color = self._make_color(0, 50, 0)
            msg.leds = [color] * 6

        self.light_pub.publish(msg)

    @staticmethod
    def _make_color(r, g, b):
        """Build a LedColor message from 0-255 RGB components."""
        c = LedColor()
        c.red = r
        c.green = g
        c.blue = b
        return c


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
