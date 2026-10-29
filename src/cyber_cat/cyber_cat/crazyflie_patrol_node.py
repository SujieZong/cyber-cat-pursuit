#!/usr/bin/env python3
"""Simulated Crazyflie patrol: takeoff, hover, then a closed square circuit.

This is the *simulation* target generator — it gives the TurtleBot something
to chase inside Ignition Gazebo. On real hardware the Crazyflie is flown by
``scripts/crazyflie_flight.py`` over cflib instead, because cflib's threading
model does not co-exist with an rclpy executor in one process.

Waypoints are laid out relative to wherever odometry first reports the drone,
so the square does not depend on the spawn pose being exact.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class CrazyfliePatrol(Node):
    """Odometry-closed-loop square patrol with proportional altitude hold."""

    def __init__(self):
        super().__init__("crazyflie_patrol")

        # Ignition bridge topics, fixed rather than parameterised: this node
        # only ever runs against the bundled simulation world.
        self.cmd_pub = self.create_publisher(
            Twist, "/model/crazyflie/cmd_vel", 10
        )

        self.create_subscription(
            Odometry, "/model/crazyflie/odometry", self.odom_cb, 10
        )

        self.create_subscription(
            String, "/crazyflie/command", self.cmd_cb, 10
        )

        self.has_odom = False
        self.stop_flag = False

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # ── Tuning ────────────────────────────────────────────────────
        self.target_z = 0.5          # m, patrol altitude

        self.alt_kp = 0.6            # altitude P gain
        self.alt_max = 0.2           # m/s, clamp on vertical speed
        self.alt_deadband = 0.02     # m, stops hunting around target_z

        self.xy_kp = 0.6             # horizontal P gain
        self.xy_max = 0.25           # m/s, clamp on horizontal speed

        self.side = 0.8              # m, square edge length
        self.wp_tol = 0.05           # m, waypoint arrival radius

        self.phase = "TAKEOFF"
        self.hover_time = 0.0

        self.waypoints = []
        self.wp_idx = 0

        self.timer = self.create_timer(0.05, self.loop)

    # ================= callbacks =================
    def cmd_cb(self, msg):
        if msg.data == "stop":
            self.stop_flag = True

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        if not self.has_odom:
            self.has_odom = True
            self.init_square()
            self.get_logger().info(
                f"Odom received. Start at ({self.x:.2f}, {self.y:.2f})"
            )

    # ================= logic =================
    def init_square(self):
        x0, y0 = self.x, self.y
        L = self.side

        self.waypoints = [
            (x0, y0),
            (x0 + L, y0),
            (x0 + L, y0 + L),
            (x0, y0 + L),
        ]
        self.wp_idx = 0

    def altitude_control(self):
        err = self.target_z - self.z

        if abs(err) < self.alt_deadband:
            return 0.0

        vz = self.alt_kp * err
        return max(-self.alt_max, min(self.alt_max, vz))

    def xy_control(self):
        tx, ty = self.waypoints[self.wp_idx]

        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        if dist < self.wp_tol:
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
            return 0.0, 0.0

        vx = self.xy_kp * dx
        vy = self.xy_kp * dy

        speed = math.hypot(vx, vy)
        if speed > self.xy_max:
            scale = self.xy_max / speed
            vx *= scale
            vy *= scale

        return vx, vy

    # ================= main loop =================
    def loop(self):
        msg = Twist()

        if not self.has_odom or self.stop_flag:
            self.cmd_pub.publish(msg)
            return

        # ===== TAKEOFF =====
        if self.phase == "TAKEOFF":
            msg.linear.z = self.altitude_control()

            if abs(self.target_z - self.z) < 0.03:
                self.get_logger().info("Reached height, switching to HOVER")
                self.phase = "HOVER"

        # ===== HOVER =====
        elif self.phase == "HOVER":
            msg.linear.z = self.altitude_control()

            self.hover_time += 0.05
            if self.hover_time > 2.0:
                self.get_logger().info("Start square")
                self.phase = "SQUARE"

        # ===== SQUARE =====
        elif self.phase == "SQUARE":
            msg.linear.z = self.altitude_control()
            vx, vy = self.xy_control()
            msg.linear.x = vx
            msg.linear.y = vy

        # safety clamp
        msg.linear.z = max(-0.25, min(0.25, msg.linear.z))

        self.cmd_pub.publish(msg)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = CrazyfliePatrol()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
