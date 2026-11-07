#!/usr/bin/env python3
"""Keyboard teleop for robot base (TwistStamped) and MeArm (Float64MultiArray)."""

import select
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Wheel keybindings mirror teleop_twist_keyboard.
WHEEL_BINDINGS = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
}

HELP = """\r
Robot keyboard teleop\r
=====================\r
Wheels:           u  i  o\r
                  j  k  l\r
                  m  ,  .\r
\r
Arm:\r
  w / x : shoulder up / down\r
  e / c : elbow    up / down\r
  r / f : claw     open / close\r
  a / d : arm base rotate CCW / CW\r
  s     : stop arm base rotation\r
  h     : home pose\r
  [ / ] : decrease / increase arm step\r
\r
Other:\r
  k / space : stop wheels (space also stops arm base)\r
  Ctrl-C    : quit\r
"""

# MeArm URDF limits, in order [mearm_joint_0, mearm_joint_1, mearm_joint_2, mearm_joint_3].
ARM_LIMITS = [(-1.0, 1.0), (-1.57, 1.0), (0.0, 1.57), (0.0, 1.57)]
ARM_HOME = [0.0, 0.0, 0.0, 1.57]


def clamp(value, lo, hi):
    return max(lo, min(value, hi))


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')

        self.stamped = self.declare_parameter('stamped', True).value
        self.frame_id = self.declare_parameter('frame_id', '').value
        self.speed = self.declare_parameter('speed', 0.5).value
        self.turn = self.declare_parameter('turn', 2.0).value
        self.speed_limit = self.declare_parameter('speed_limit', 1.0).value
        self.turn_limit = self.declare_parameter('turn_limit', 3.0).value
        self.arm_step = self.declare_parameter('arm_step', 0.05).value
        self.arm_base_speed = self.declare_parameter('arm_base_speed', 0.5).value
        self.arm_base_timeout = self.declare_parameter('arm_base_timeout', 0.5).value

        TwistMsg = TwistStamped if self.stamped else Twist
        self.cmd_vel_pub = self.create_publisher(TwistMsg, 'cmd_vel', 10)
        self.arm_pub = self.create_publisher(
            Float64MultiArray, '/joint_group_position_controller/commands', 10)

        self.lock = threading.RLock()
        with self.lock:
            self.x = 0.0
            self.th = 0.0
            self.arm = list(ARM_HOME)
            self.last_base_key_time = self.get_clock().now()

        # Auto-stop the arm base if no a/d/s key arrives for a while.
        self.create_timer(0.05, self._timer_callback)

    def _timer_callback(self):
        with self.lock:
            if self.arm[0] != 0.0:
                elapsed = (self.get_clock().now() - self.last_base_key_time).nanoseconds * 1e-9
                if elapsed > self.arm_base_timeout:
                    self.arm[0] = 0.0
                    self._publish_arm_locked()

    def handle_key(self, key):
        with self.lock:
            now = self.get_clock().now()
            if key in WHEEL_BINDINGS:
                self.x, _, _, self.th = WHEEL_BINDINGS[key]
                self._publish_twist_locked()
            elif key == ' ':
                # Stop wheels and arm base.
                self.x = 0.0
                self.th = 0.0
                self.arm[0] = 0.0
                self._publish_twist_locked()
                self._publish_arm_locked()
                self.last_base_key_time = now
            elif key == 'k':
                self.x = 0.0
                self.th = 0.0
                self._publish_twist_locked()
            elif key == 's':
                self.arm[0] = 0.0
                self._publish_arm_locked()
                self.last_base_key_time = now
            elif key == 'h':
                self.arm = list(ARM_HOME)
                self._publish_arm_locked()
                self.last_base_key_time = now
            elif key == 'w':
                self.arm[1] = clamp(self.arm[1] + self.arm_step, *ARM_LIMITS[1])
                self._publish_arm_locked()
            elif key == 'x':
                self.arm[1] = clamp(self.arm[1] - self.arm_step, *ARM_LIMITS[1])
                self._publish_arm_locked()
            elif key == 'e':
                self.arm[2] = clamp(self.arm[2] + self.arm_step, *ARM_LIMITS[2])
                self._publish_arm_locked()
            elif key == 'c':
                self.arm[2] = clamp(self.arm[2] - self.arm_step, *ARM_LIMITS[2])
                self._publish_arm_locked()
            elif key == 'r':
                self.arm[3] = clamp(self.arm[3] + self.arm_step, *ARM_LIMITS[3])
                self._publish_arm_locked()
            elif key == 'f':
                self.arm[3] = clamp(self.arm[3] - self.arm_step, *ARM_LIMITS[3])
                self._publish_arm_locked()
            elif key == 'a':
                self.arm[0] = -self.arm_base_speed
                self._publish_arm_locked()
                self.last_base_key_time = now
            elif key == 'd':
                self.arm[0] = self.arm_base_speed
                self._publish_arm_locked()
                self.last_base_key_time = now
            elif key == '[':
                self.arm_step = max(0.01, self.arm_step - 0.01)
                print(f'\rarm step: {self.arm_step:.2f} rad\r')
            elif key == ']':
                self.arm_step += 0.01
                print(f'\rarm step: {self.arm_step:.2f} rad\r')
            else:
                # Unknown key: stop wheels, leave arm pose unchanged.
                self.x = 0.0
                self.th = 0.0
                self._publish_twist_locked()

    def _publish_twist_locked(self):
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            if self.frame_id:
                msg.header.frame_id = self.frame_id
            twist = msg.twist
        else:
            msg = Twist()
            twist = msg
        twist.linear.x = clamp(self.x * self.speed, -self.speed_limit, self.speed_limit)
        twist.angular.z = clamp(self.th * self.turn, -self.turn_limit, self.turn_limit)
        self.cmd_vel_pub.publish(msg)

    def _publish_arm_locked(self):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in self.arm]
        self.arm_pub.publish(msg)

    def stop_all(self):
        with self.lock:
            self.x = 0.0
            self.th = 0.0
            self.arm[0] = 0.0
            self._publish_twist_locked()
            self._publish_arm_locked()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    try:
        sys.stdout.write(HELP)
        sys.stdout.flush()
        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if ready:
                key = sys.stdin.read(1)
                if key == '\x03':  # Ctrl-C
                    break
                if key:
                    node.handle_key(key)
    finally:
        node.stop_all()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        spinner.join()


if __name__ == '__main__':
    main()
