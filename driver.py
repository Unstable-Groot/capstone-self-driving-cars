#!/usr/bin/env python

import os
import struct
import threading

import can
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int64

# Constants
POSE_TOPIC      = os.environ.get("POSE_TOPIC", "/zed/zed_node/pose")
FPS             = 40
STOP_TIME       = 3
PARKED_FRAMES   = FPS
STILL_THRESHOLD = 0.01
BRAKE_THROTTLE  = -70

# State
throttle         = 0
steer            = 0
stop_signal      = 0
parked_countdown = 0
auto_throttle    = 0
auto_steer       = 0
toggle           = 0
pose_dict        = None
prev_pose_dict   = None

# Subscriber callbacks
def stop_callback(data):
    global stop_signal
    stop_signal = data.data[0]


def steering_callback(data):
    global steer
    steer = data.data


def throttle_callback(data):
    global throttle
    throttle = data.data


def auto_steering_callback(data):
    global auto_steer
    auto_steer = data.data


def auto_throttle_callback(data):
    global auto_throttle
    auto_throttle = data.data


def toggle_callback(data):
    global toggle
    toggle = data.data


def pos_callback(msg):
    global pose_dict, prev_pose_dict

    prev_pose_dict = pose_dict

    p     = msg.pose.position
    o     = msg.pose.orientation
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    pose_dict = {
        "type":        "pose",
        "ts":          stamp,
        "frame":       msg.header.frame_id,
        "position":    {"x": p.x, "y": p.y, "z": p.z},
        "orientation": {"x": o.x, "y": o.y, "z": o.z, "w": o.w},
    }


def is_car_still_or_going_backwards() -> bool:
    """Return True if the car is not making forward progress."""
    if pose_dict is None or prev_pose_dict is None:
        return True  # No data yet

    dt = pose_dict["ts"] - prev_pose_dict["ts"]
    if dt <= 0:
        return True  # Bad timestamp delta — assume still

    dx = pose_dict["position"]["x"] - prev_pose_dict["position"]["x"]
    dy = pose_dict["position"]["y"] - prev_pose_dict["position"]["y"]

    # Rotate unit forward vector [1, 0, 0] by quaternion to get car's heading
    qx = pose_dict["orientation"]["x"]
    qy = pose_dict["orientation"]["y"]
    qz = pose_dict["orientation"]["z"]
    qw = pose_dict["orientation"]["w"]

    forward_x = 1 - 2 * (qy**2 + qz**2)
    forward_y = 2 * (qx * qy + qw * qz)

    # Project displacement onto forward axis
    forward_speed = (dx * forward_x + dy * forward_y) / dt
    print(f"forward_speed: {forward_speed:.4f} m/s")

    return forward_speed < STILL_THRESHOLD


def main(args=None):
    global throttle, steer, toggle, parked_countdown

    bus = can.interface.Bus(bustype="socketcan", channel="can0", bitrate=250000)

    rclpy.init(args=args)
    node = Node("driver")

    # Subscriptions
    node.create_subscription(Int64,          "/manual_throttle",  throttle_callback,      1)
    node.create_subscription(Int64,          "/manual_steering",  steering_callback,       1)
    node.create_subscription(Int64,          "/auto_throttle",    auto_throttle_callback,  1)
    node.create_subscription(Int64,          "/auto_steering",    auto_steering_callback,  1)
    node.create_subscription(Int32MultiArray, "/detections/stop", stop_callback,           1)
    node.create_subscription(Int64,          "/manual_a_toggle",  toggle_callback,         1)
    node.create_subscription(PoseStamped,    POSE_TOPIC,          pos_callback,            1)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    stopping    = False
    temp_ignore = 0

    rate = node.create_rate(FPS, node.get_clock())
    while rclpy.ok():
        try:
            current_throttle = 0
            current_steer    = 0

            if toggle == 0:
                # --- MANUAL MODE ---
                current_throttle = throttle
                current_steer    = steer
                # Reset auto-states so the car doesn't jump when toggling back
                stopping         = False
                parked_countdown = 0

            else:
                # --- AUTO MODE ---
                current_throttle = throttle
                current_steer    = auto_steer

                # Trigger a stop when a stop sign is detected
                if (stop_signal == 1
                        and current_throttle > 0
                        and not stopping
                        and parked_countdown == 0
                        and temp_ignore == 0):
                    stopping    = True
                    temp_ignore = 1

                # Re-arm the stop trigger only after the sign clears
                if stop_signal == 0 and not stopping and parked_countdown == 0:
                    temp_ignore = 0

                if parked_countdown > 0:
                    current_throttle  = 0
                    current_steer     = 0
                    parked_countdown -= 1

                elif stopping:
                    if not is_car_still_or_going_backwards():
                        # Still moving —> brake
                        current_throttle = BRAKE_THROTTLE
                        current_steer    = auto_steer
                    else:
                        current_throttle = 0
                        stopping         = False
                        parked_countdown = PARKED_FRAMES

            can_data = struct.pack(">hhI", int(current_throttle), int(current_steer), 0)
            msg      = can.Message(arbitration_id=0x1, data=can_data, is_extended_id=False)
            bus.send(msg)

        except Exception as error:
            print("An exception occurred:", error)
        finally:
            rate.sleep()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
