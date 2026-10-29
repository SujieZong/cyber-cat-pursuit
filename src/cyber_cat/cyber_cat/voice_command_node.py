###############################################################################
# FILE: cyber_cat/voice_command_node.py
#
# Operator command input: reads typed commands and publishes /voice/command.
#
# Named "voice" because the topic is the seam the original design reserved for
# speech-to-text. Nothing downstream cares where a command came from, so a
# Whisper front-end can replace this node without touching the state machine.
# Shipping the keyboard version first kept the demo deterministic.
#
# stdin is polled non-blockingly from a ROS timer rather than read on a worker
# thread: a blocking readline() would stall the single-threaded executor and
# stop the node from ever spinning.
###############################################################################
import sys
import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoiceCommand(Node):
    """Publish operator commands (``start`` / ``stop``) to /voice/command."""

    def __init__(self):
        super().__init__('voice_command')
        self.cmd_pub = self.create_publisher(String, '/voice/command', 10)

        # Check stdin at 10 Hz
        self.timer = self.create_timer(0.1, self._check_stdin)

        self.get_logger().info(
            'Command input ready — type "start" or "stop", then Enter'
        )
        print('[command] > ', end='', flush=True)

    def _check_stdin(self):
        """Non-blocking check for keyboard input."""
        if select.select([sys.stdin], [], [], 0.0)[0]:
            line = sys.stdin.readline().strip()
            if not line:
                return

            msg = String()
            msg.data = line
            self.cmd_pub.publish(msg)
            self.get_logger().info(f'Published command: "{line}"')
            print('[command] > ', end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
