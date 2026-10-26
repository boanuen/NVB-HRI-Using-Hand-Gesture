#!/usr/bin/env python3
"""Subscribe /detected_gesture → control TurtleBot 4 via /cmd_vel."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
import time

# Movement parameters
LIN_SPEED = 0.2   # m/s
ANG_SPEED = 0.8   # rad/s


class BehaviorNode(Node):
    # States
    IDLE = 'IDLE'
    SELECTED = 'SELECTED'
    EXECUTING = 'EXECUTING'

    def __init__(self):
        super().__init__('robot_behavior_node')
        self.sub = self.create_subscription(
            String, '/detected_gesture', self.on_gesture, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.state = self.IDLE
        self.selected_behavior = None
        self.exec_start = 0.0
        self.exec_step = 0
        self.last_gesture_time = time.time()
        self.GESTURE_TIMEOUT = 10.0  # stop if no gesture for 10s

        # Timer for continuous motion commands at 10 Hz
        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info('Behavior node ready — waiting for gestures')

    def on_gesture(self, msg):
        g = msg.data
        self.last_gesture_time = time.time()
        self.get_logger().info(f'Received: {g} | State: {self.state} | Selected: {self.selected_behavior}')

        # Open Palm → emergency stop (always works)
        if g == 'Open Palm':
            self.stop_robot()
            self.state = self.IDLE
            self.selected_behavior = None
            self.get_logger().info('STOP — robot halted')
            return

        # Closed Fist → cancel current action
        if g == 'Closed Fist':
            self.stop_robot()
            self.state = self.IDLE
            self.selected_behavior = None
            self.get_logger().info('CANCEL — back to idle')
            return

        # Wave → immediate greeting (no confirmation needed)
        if g == 'Wave':
            self.stop_robot()
            self.state = self.EXECUTING
            self.selected_behavior = 'wave'
            self.exec_start = time.time()
            self.exec_step = 0
            self.get_logger().info('WAVE — greeting!')
            return

        # Select behaviors (only when idle or already selected)
        if self.state in (self.IDLE, self.SELECTED):
            if g == 'Pointing Up':
                self.selected_behavior = 'forward_back'
                self.state = self.SELECTED
                self.get_logger().info('Selected: Forward-Back x4')
                return
            if g == 'Victory':
                self.selected_behavior = 'circle'
                self.state = self.SELECTED
                self.get_logger().info('Selected: Circle')
                return

        # Thumbs Up → confirm and execute
        if g == 'Thumbs Up' and self.state == self.SELECTED:
            self.state = self.EXECUTING
            self.exec_start = time.time()
            self.exec_step = 0
            self.get_logger().info(f'CONFIRMED — executing: {self.selected_behavior}')
            return

    def update(self):
        """Called at 10 Hz to send movement commands."""
        # Safety: stop if no gesture received for too long
        if (self.state == self.EXECUTING
                and time.time() - self.last_gesture_time > self.GESTURE_TIMEOUT):
            self.get_logger().warn('No gesture signal — safety stop')
            self.stop_robot()
            self.state = self.IDLE
            self.selected_behavior = None
            return

        if self.state != self.EXECUTING:
            return

        t = time.time() - self.exec_start

        if self.selected_behavior == 'wave':
            self.exec_wave(t)
        elif self.selected_behavior == 'forward_back':
            self.exec_forward_back(t)
        elif self.selected_behavior == 'circle':
            self.exec_circle(t)

    def exec_wave(self, t):
        """Rotate left-right to greet. ~4 seconds total."""
        twist = Twist()
        if t < 4.0:
            twist.angular.z = ANG_SPEED * math.sin(2 * math.pi * t)
        else:
            self.finish_behavior()
            return
        self.pub.publish(twist)

    def exec_forward_back(self, t):
        """Forward-back 4 times. Each cycle: 1s fwd + 1s back = 2s, total 8s."""
        twist = Twist()
        if t < 8.0:
            cycle_t = t % 2.0
            twist.linear.x = LIN_SPEED if cycle_t < 1.0 else -LIN_SPEED
        else:
            self.finish_behavior()
            return
        self.pub.publish(twist)

    def exec_circle(self, t):
        """Move in a circle. ~6 seconds for one full loop."""
        twist = Twist()
        if t < 6.0:
            twist.linear.x = LIN_SPEED
            twist.angular.z = ANG_SPEED * 0.6
        else:
            self.finish_behavior()
            return
        self.pub.publish(twist)

    def finish_behavior(self):
        self.stop_robot()
        self.get_logger().info(f'Finished: {self.selected_behavior}')
        self.state = self.IDLE
        self.selected_behavior = None

    def stop_robot(self):
        self.pub.publish(Twist())

    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
