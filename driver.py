#!/usr/bin/env python
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseStamped
import struct
import can
import threading

POSE_TOPIC = os.environ.get("POSE_TOPIC", "/zed/zed_node/pose")

FPS = 40
throttle = 0
steer = 0
stop_signal = 0

stop_time = 3
parked_frames = FPS
parked_countdown = 0

auto_throttle = 0
auto_steer = 0
 
toggle = 0

pose_dict = None
prev_pose_dict = None

## ------------
## YOUR CODE
## ------------
def stop_callback(data):
    global stop_signal
    # print("################################",data.data)
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

    p = msg.pose.position
    o = msg.pose.orientation
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    pose_dict = {
        "type": "pose",
        "ts": stamp,
        "frame": msg.header.frame_id,
        "position": {"x": p.x, "y": p.y, "z": p.z},
        "orientation": {"x": o.x, "y": o.y, "z": o.z, "w": o.w},
    }

STILL_THRESHOLD = 0.01
def is_car_still_or_going_backwards() -> bool:
    if pose_dict is None or prev_pose_dict is None:
        return True # No data yet

    dt = pose_dict["ts"] - prev_pose_dict["ts"]
    if dt <= 0:
        return True  # Bad timestamp delta, assume still

    # Compute displacement between the two poses
    dx = pose_dict["position"]["x"] - prev_pose_dict["position"]["x"]
    dy = pose_dict["position"]["y"] - prev_pose_dict["position"]["y"]

    # Use orientation to project displacement onto the car's forward axis,
    # so lateral drift or noise doesn't count as "moving forward"
    qx = pose_dict["orientation"]["x"]
    qy = pose_dict["orientation"]["y"]
    qz = pose_dict["orientation"]["z"]
    qw = pose_dict["orientation"]["w"]

    # Rotate unit forward vector [1, 0, 0] by quaternion to get car's heading
    forward_x = 1 - 2 * (qy**2 + qz**2)
    forward_y = 2 * (qx*qy + qw*qz)

    # Project displacement onto forward axis (dot product)
    forward_speed = (dx * forward_x + dy * forward_y) / dt

    print(f"forward_speed: {forward_speed:.4f} m/s")

    return forward_speed < STILL_THRESHOLD


def main(args=None):
    global throttle, parked_countdown, steer, toggle

    bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate= 250000)
    
    rclpy.init(args=args)
    node = Node("driver")

    ## ------------
    ## YOUR CODE
    # Subscribe to the manual_throttle and manual_steering commands
    ## ------------

    manual_throttle_sub = node.create_subscription(Int64, "/manual_throttle", throttle_callback, 1)
    manual_steering_sub = node.create_subscription(Int64, "/manual_steering", steering_callback, 1)

    auto_throttle_sub = node.create_subscription(Int64, "/auto_throttle", auto_throttle_callback, 1)
    auto_steering_sub = node.create_subscription(Int64, "/auto_steering", auto_steering_callback, 1)

    stop_sub = node.create_subscription(Int32MultiArray, "/detections/stop", stop_callback, 1)
    toggle_sub = node.create_subscription(Int64, '/manual_a_toggle', toggle_callback, 1)

    pos_sub = node.create_subscription(PoseStamped, POSE_TOPIC, pos_callback, 1)


    thread = threading.Thread(target=rclpy. spin, args=(node, ), daemon=True)
    thread.start()

    stopping = False
    temp_ignore = 0
 
    rate = node.create_rate(FPS, node.get_clock())
    while rclpy.ok():

        try:
            current_throttle = 0
            current_steer = 0
            
            if toggle == 0:
                # --- MANUAL MODE ---
                # Direct bypass: use manual inputs only
                current_throttle = throttle
                current_steer = steer
                # Reset auto-states so it doesn't "jump" when you toggle back
                stopping = False
                parked_countdown = 0
            
            else:
                # --- AUTO MODE ---
                current_throttle = throttle
                current_steer = auto_steer

                # Check for stop sign
                if stop_signal == 1 and current_throttle > 0 and not stopping and parked_countdown == 0 and temp_ignore == 0:
                    stopping = True
                    temp_ignore = 1

                # We only want to stop the car again after we pass the current stop sign
                if stop_signal == 0 and not stopping and parked_countdown == 0:
                    temp_ignore = 0

                if parked_countdown > 0:
                    current_throttle = 0
                    current_steer = 0
                    parked_countdown -= 1

                elif stopping:
                    if not is_car_still_or_going_backwards():
                        # Still moving toward the sign: Apply Reverse/Brake
                        current_throttle = -70
                        current_steer = auto_steer # Keep steering while braking
                    else:
                        current_throttle = 0
                        stopping = False
                        parked_countdown = parked_frames

            can_data = struct.pack('>hhI', int(current_throttle), int(current_steer), 0)
            msg = can.Message(arbitration_id=0x1, data=can_data, is_extended_id=False)
            bus.send(msg)


        except Exception as error:
            print("An exception occurred:", error)
        finally:
            rate.sleep()

    rclpy.spin(node)
    rclpy.shutdown()

	
if __name__ == '__main__':
	main()
