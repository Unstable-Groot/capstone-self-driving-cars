#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int64
import cv2
from cv_bridge import CvBridge
import threading
import numpy as np

# --- CONSTANTS ---
IMAGE_TOPIC = '/zed/zed_node/right/color/rect/image'
HEIGHT, WIDTH = 540, 960
HALF_WIDTH = WIDTH // 2
AVERAGE_LINE_WIDTH = 340 # The "imaginary" distance to the center if 1 line is seen

# --- GLOBAL STATES ---
PREVIOUS_DECISIONS = [0] * 5
OLDEST_DECISION = 0
image_raw = None
br = CvBridge()

def camera_callback(data):
    global image_raw
    try:
        # 'bgr8' is standard for OpenCV. 
        # If this fails, the topic name might be wrong or the camera isn't running.
        image_raw = br.imgmsg_to_cv2(data, desired_encoding='bgr8')
    except Exception as e:
        print(f"CVBridge Error: {e}")

def is_line_valid(fit, side):
    if fit is None: return False
    m, b = fit
    x_base = m * HEIGHT + b # x position at the bottom of the image
    
    # 1. Slope check: If |m| > 1.5, the line is too horizontal (likely noise)
    if abs(m) > 1.5: return False
    
    # 2. Side check: Prevent left lines from appearing on the far right
    buffer = 80
    if side == 'left' and x_base > (HALF_WIDTH + buffer): return False
    if side == 'right' and x_base < (HALF_WIDTH - buffer): return False
    return True

def process_image(img):
    global OLDEST_DECISION, PREVIOUS_DECISIONS
    
    # 1. Pre-process
    # Broadened HSV range to prevent "Black Image" issues
    lower_yellow = np.array([10, 40, 90]) 
    upper_yellow = np.array([50, 255, 255])
    
    hsv = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    # ROI Mask (Triangle/Trapezoid)
    roi_mask = np.zeros_like(mask)
    vertices = np.array([[ (0, HEIGHT), (0, HEIGHT//2), (WIDTH//2, HEIGHT//2 - 50), (WIDTH, HEIGHT//2), (WIDTH, HEIGHT) ]], dtype=np.int32)
    cv2.fillPoly(roi_mask, vertices, 255)
    masked = cv2.bitwise_and(mask, roi_mask)

    # 2. Clustering
    pixel_y, pixel_x = np.where(masked > 0)
    if len(pixel_x) < 30: 
        cv2.imshow('Debug View', img) # Show raw if nothing detected
        return None

    left_x, left_y = pixel_x[pixel_x < HALF_WIDTH], pixel_y[pixel_x < HALF_WIDTH]
    right_x, right_y = pixel_x[pixel_x >= HALF_WIDTH], pixel_y[pixel_x >= HALF_WIDTH]

    # 3. Fitting
    l_fit = np.polyfit(left_y, left_x, 1) if len(left_y) > 50 else None
    r_fit = np.polyfit(right_y, right_x, 1) if len(right_y) > 50 else None

    # Validate
    if not is_line_valid(l_fit, 'left'): l_fit = None
    if not is_line_valid(r_fit, 'right'): r_fit = None

    # 4. Target Midline
    y_lookahead = HEIGHT // 2 + 100
    def get_x(fit, y): return int(fit[0] * y + fit[1])

    if l_fit is not None and r_fit is not None:
        target_x = (get_x(l_fit, y_lookahead) + get_x(r_fit, y_lookahead)) // 2
    elif l_fit is not None:
        target_x = get_x(l_fit, y_lookahead) + (AVERAGE_LINE_WIDTH // 2)
    elif r_fit is not None:
        target_x = get_x(r_fit, y_lookahead) - (AVERAGE_LINE_WIDTH // 2)
    else:
        return None

    # 5. Steering
    cte = HALF_WIDTH - target_x
    steering = np.clip((cte / (WIDTH / 4)) * 80.0, -80, 80)
    
    PREVIOUS_DECISIONS[OLDEST_DECISION] = steering
    OLDEST_DECISION = (OLDEST_DECISION + 1) % len(PREVIOUS_DECISIONS)

    # 6. Draw
    viz = img.copy()
    cv2.circle(viz, (target_x, y_lookahead), 10, (255, 0, 255), -1)
    cv2.imshow('Debug View', viz)
    cv2.waitKey(1)
    
    return sum(PREVIOUS_DECISIONS) / len(PREVIOUS_DECISIONS)

def main():
    rclpy.init()
    node = rclpy.create_node('lane_follower_final')
    node.create_subscription(Image, IMAGE_TOPIC, camera_callback, 10)
    
    # Run ROS loop in background
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    pub = node.create_publisher(Int64, "/auto_steering", 1)
    rate = node.create_rate(15)

    print("Checking for camera stream...")
    while rclpy.ok():
        if image_raw is not None:
            result = process_image(image_raw)
            if result is not None:
                # Apply hardware correction (+6) and negation (-)
                final_steer = -(result + 6)
                msg = Int64()
                msg.data = int(final_steer)
                pub.publish(msg)
        else:
            print("Still waiting for image...")
        
        rate.sleep()

if __name__ == '__main__':
    main()