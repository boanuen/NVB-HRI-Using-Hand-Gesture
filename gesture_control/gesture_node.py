#!/usr/bin/env python3
"""Webcam hand gesture recognition → publishes to /detected_gesture."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import mediapipe as mp
import numpy as np
import time
import math

# Landmark indices
WRIST = 0
THUMB_CMC, THUMB_MCP, THUMB_IP, THUMB_TIP = 1, 2, 3, 4
INDEX_MCP, INDEX_PIP, INDEX_DIP, INDEX_TIP = 5, 6, 7, 8
MIDDLE_MCP, MIDDLE_PIP, MIDDLE_DIP, MIDDLE_TIP = 9, 10, 11, 12
RING_MCP, RING_PIP, RING_DIP, RING_TIP = 13, 14, 15, 16
PINKY_MCP, PINKY_PIP, PINKY_DIP, PINKY_TIP = 17, 18, 19, 20

FINGER_TIPS = [THUMB_TIP, INDEX_TIP, MIDDLE_TIP, RING_TIP, PINKY_TIP]
FINGER_PIPS = [THUMB_IP, INDEX_PIP, MIDDLE_PIP, RING_PIP, PINKY_PIP]
FINGER_MCPS = [THUMB_MCP, INDEX_MCP, MIDDLE_MCP, RING_MCP, PINKY_MCP]


def dist(a, b):
    return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)


def get_finger_states(lm, handedness):
    """Robust finger detection using distance ratios."""
    fingers = []

    # Thumb: tip-to-wrist vs mcp-to-wrist (extended if tip is farther)
    thumb_tip_d = dist(lm[THUMB_TIP], lm[INDEX_MCP])
    thumb_mcp_d = dist(lm[THUMB_MCP], lm[INDEX_MCP])
    fingers.append(thumb_tip_d > thumb_mcp_d * 1.2)

    # 4 fingers: tip closer to wrist = curled, tip far from wrist = extended
    for tip, pip, mcp in [(INDEX_TIP, INDEX_PIP, INDEX_MCP),
                          (MIDDLE_TIP, MIDDLE_PIP, MIDDLE_MCP),
                          (RING_TIP, RING_PIP, RING_MCP),
                          (PINKY_TIP, PINKY_PIP, PINKY_MCP)]:
        tip_to_wrist = dist(lm[tip], lm[WRIST])
        mcp_to_wrist = dist(lm[mcp], lm[WRIST])
        fingers.append(tip_to_wrist > mcp_to_wrist * 1.1)

    return fingers


def is_thumb_up(lm, fingers):
    """Thumb pointing upward + other fingers curled."""
    if not fingers[0]:
        return False
    if sum(fingers[1:]) > 1:
        return False
    # Thumb tip must be above thumb mcp (y decreases upward)
    return lm[THUMB_TIP].y < lm[THUMB_MCP].y - 0.04


def classify(lm, handedness):
    """Classify hand gesture from landmarks."""
    fingers = get_finger_states(lm, handedness)
    n_up = sum(fingers)

    # Closed Fist: all fingers curled
    if n_up == 0:
        return 'Closed Fist'

    # Open Palm: all fingers extended + fingers spread
    if n_up >= 4 and fingers[1] and fingers[2] and fingers[3]:
        if n_up == 5:
            return 'Open Palm'
        # 4 fingers up (thumb may be ambiguous) — still Open Palm
        return 'Open Palm'

    # Thumbs Up: only thumb extended + pointing up
    if is_thumb_up(lm, fingers):
        return 'Thumbs Up'

    # Pointing Up: only index extended
    if fingers[1] and n_up <= 2 and not fingers[2] and not fingers[3] and not fingers[4]:
        return 'Pointing Up'

    # Victory: index + middle extended, others curled
    if fingers[1] and fingers[2] and not fingers[3] and not fingers[4]:
        return 'Victory'

    return None  # ambiguous — ignore


class GestureNode(Node):
    def __init__(self):
        super().__init__('gesture_recognition_node')
        self.pub = self.create_publisher(String, '/detected_gesture', 10)

        self.declare_parameter('camera_id', 0)
        cam_id = self.get_parameter('camera_id').value

        self.cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(
                f'Cannot open webcam (id={cam_id})! '
                'Ensure webcam is mounted in WSL2 via usbipd.')
            raise RuntimeError('Webcam not available')

        self.cap.set(cv2.CAP_PROP_FOURCC,
                     cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 15)

        self.hands = mp.solutions.hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.6,
        )
        self.mp_draw = mp.solutions.drawing_utils

        self.show_gui = True
        try:
            cv2.namedWindow('Gesture Recognition', cv2.WINDOW_NORMAL)
        except cv2.error:
            self.show_gui = False
            self.get_logger().warn('No display — running headless')

        # Debounce: gesture must be stable for STABLE_FRAMES before publishing
        self.stable_gesture = None
        self.gesture_count = 0
        self.last_published = 'None'
        self.STABLE_FRAMES = 8  # ~0.5s at 15fps

        # Cooldown: min time between different gesture publishes
        self.last_publish_time = 0.0
        self.COOLDOWN = 1.0  # seconds

        # Wave detection
        self.wave_xs = []
        self.wave_time = 0.0

        # Display state
        self.display_gesture = 'None'
        self.display_score = 0.0

        self.timer = self.create_timer(1.0 / 15, self.process_frame)
        self.get_logger().info(f'Gesture node started (cam={cam_id}, 15 FPS)')

    def detect_wave(self, lm):
        """Detect wave: open hand moving left-right rapidly."""
        wx = lm[WRIST].x
        now = time.time()

        if now - self.wave_time > 2.0:
            self.wave_xs.clear()
        self.wave_time = now

        self.wave_xs.append(wx)
        if len(self.wave_xs) > 20:
            self.wave_xs.pop(0)
        if len(self.wave_xs) < 10:
            return False

        # Count direction reversals with significant movement
        changes = 0
        for i in range(2, len(self.wave_xs)):
            d1 = self.wave_xs[i-1] - self.wave_xs[i-2]
            d2 = self.wave_xs[i] - self.wave_xs[i-1]
            if d1 * d2 < 0 and abs(d1) > 0.015:
                changes += 1
        return changes >= 3

    def process_frame(self):
        ok, frame = self.cap.read()
        if not ok:
            return

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        gesture = None  # None = ambiguous/no hand, don't change state

        if results.multi_hand_landmarks:
            hl = results.multi_hand_landmarks[0]
            lm = hl.landmark
            hand_label = results.multi_handedness[0].classification[0].label
            self.display_score = results.multi_handedness[0].classification[0].score

            self.mp_draw.draw_landmarks(
                frame, hl, mp.solutions.hands.HAND_CONNECTIONS)

            fingers = get_finger_states(lm, hand_label)

            # Wave: open hand + oscillation
            if sum(fingers) >= 4 and self.detect_wave(lm):
                gesture = 'Wave'
            else:
                gesture = classify(lm, hand_label)
        else:
            self.wave_xs.clear()
            self.display_score = 0.0
            gesture = 'None'  # no hand visible = explicit None

        # Skip ambiguous frames (gesture is Python None)
        if gesture is None:
            self._draw(frame)
            return

        self.display_gesture = gesture

        # Debounce: must see same gesture for STABLE_FRAMES consecutively
        if gesture == self.stable_gesture:
            self.gesture_count += 1
        else:
            self.stable_gesture = gesture
            self.gesture_count = 1

        now = time.time()
        if (self.gesture_count >= self.STABLE_FRAMES
                and gesture != self.last_published
                and now - self.last_publish_time >= self.COOLDOWN):
            msg = String()
            msg.data = gesture
            self.pub.publish(msg)
            self.last_published = gesture
            self.last_publish_time = now
            self.get_logger().info(f'Published: {gesture}')

        self._draw(frame)

    def _draw(self, frame):
        if not self.show_gui:
            return
        g = self.display_gesture
        color = (0, 255, 0) if g not in ('None', None) else (128, 128, 128)
        cv2.putText(frame, f'Gesture: {g}', (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
        if self.display_score > 0:
            cv2.putText(frame, f'Conf: {self.display_score:.0%}', (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        state_color = (0, 200, 255) if self.last_published not in ('None',) else (150, 150, 150)
        cv2.putText(frame, f'Active: {self.last_published}', (10, 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)
        # Debounce progress bar
        progress = min(self.gesture_count / self.STABLE_FRAMES, 1.0)
        bar_w = int(200 * progress)
        cv2.rectangle(frame, (10, 110), (210, 125), (50, 50, 50), -1)
        cv2.rectangle(frame, (10, 110), (10 + bar_w, 125),
                      (0, 255, 0) if progress >= 1.0 else (0, 165, 255), -1)

        cv2.imshow('Gesture Recognition', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        self.hands.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
