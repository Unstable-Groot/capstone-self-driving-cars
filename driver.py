#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from std_msgs.msg import Int32,Int32MultiArray
import struct
import can
import threading
import time

fps = 40
throttle = 0
steer = 0
stop_signal = 0
current_size = 0
stop_time = 3
parked_frames = fps * 2
parked_countdown = 0

auto_throttle = 0
auto_steer = 0
 
toggle = 0

## ------------
## YOUR CODE
## ------------
def stop_callback(data):
    global stop_signal, current_size
    print("################################",data.data)
    stop_signal = data.data[0]
    current_size = data.data[1]

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



    thread = threading.Thread(target=rclpy. spin, args=(node, ), daemon=True)
    thread.start()

    stopping = False
    previous_size = 0
 
    rate = node.create_rate(fps, node.get_clock())
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
                # Default to lane following
                current_throttle = throttle
                current_steer = auto_steer

                # 1. Check for Stop Sign to trigger the sequence
                if stop_signal == 1 and current_throttle > 0 and not stopping and parked_countdown == 0:
                    stopping = True
                    print("Stop signal detected! Starting parking sequence...")

                # 2. Execution of the sequence (The Overrides)
                if parked_countdown > 0:
                    # Stage 3: Currently "Parked" (Hard Brake & Turn)
                    current_throttle = 0
                    current_steer = 0
                    parked_countdown -= 1
                    print(f"Parked. Countdown: {parked_countdown}")

                elif stopping:
                    # Stage 2: Approaching/Braking
                    if current_size > previous_size:
                        # Still moving toward the sign: Apply Reverse/Brake
                        current_throttle = -70
                        current_steer = auto_steer # Keep steering while braking
                        print(f"Braking... Size: {current_size}")
                    else:
                        # Size stopped increasing: We have reached the stop point
                        current_throttle = 0
                        stopping = False
                        parked_countdown = parked_frames
                        print("Stop reached. Entering parked state.")

            # Update size for the next iteration
            previous_size = current_size


            print("throttle:", current_throttle, "steer:", current_steer)
            # 3. Pack and Send CAN Message
            # Ensure values are integers for struct.pack
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