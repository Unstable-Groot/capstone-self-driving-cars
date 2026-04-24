#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int64
import cv2
from cv_bridge import CvBridge
import threading
import time
import math
import numpy as np

#Default: stereo image (from both L and R lenses)
IMAGE_TOPIC = '/zed/zed_node/right/color/rect/image'
STEER_TOPIC = "/auto_steering"

steering = 0
HEIGHT = int(540)
WIDTH = int(960)

IMG_HEIGHT = HEIGHT
IMG_WIDTH = int(WIDTH)

HALF_WIDTH = int(IMG_WIDTH / 2)


MASK = np.zeros((IMG_HEIGHT, IMG_WIDTH), dtype=np.uint8)
mask_edge = IMG_HEIGHT - int(IMG_HEIGHT/3)
mask_tip_x = int(IMG_WIDTH/2)
mask_tip_y = int(IMG_HEIGHT/2) + int(IMG_HEIGHT/6) 

PREVIOUS_DECISION = 0

AVERAGE_LINE_WIDTH = 300


vertices = [ np.array([ [0,IMG_HEIGHT], [0,mask_edge], [mask_tip_x,mask_tip_y], [IMG_WIDTH,mask_edge], [IMG_WIDTH,IMG_HEIGHT] ], dtype=np.int32) ]
cv2.fillPoly(MASK, vertices, (255,255,255))

br = CvBridge()
image = None
def camera_callback(data):
    global image
    image = br.imgmsg_to_cv2(data)
    image = image[:,:,:3]

def repack(line_list):
    x_list = []
    y_list = []
    for line in line_list:
        for x1, y1, x2, y2 in line:
            x_list.append(x1)
            x_list.append(x2)
            y_list.append(y1)
            y_list.append(y2)
    
    return (x_list, y_list)

def drawline(img, color, line):
    line_thickness=3
    dot_size = 3
    for x1, y1, x2, y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), color, line_thickness)
            cv2.circle(img, (x1, y1), dot_size, color, -1)
            cv2.circle(img, (x2, y2), dot_size, color, -1)

def get_x(fit, y):
    # fit[0] is the slope, fit[1] is the intercept
    # returns the horizontal pixel coordinate (x) for a given vertical pixel (y)
    return int(fit[0] * y + fit[1])

def process_image(img):
    global steering
    global PREVIOUS_DECISIONS
    global OLDEST_DECISION
    ######
    # YOUR CODE: Lane Detection
    ######

    #Example: If you want to use a smaller image
    # img = cv2.resize(img, (WIDTH, HEIGHT))
    # cv2.imshow('Small', img)


    # Yellow pass filter
    lower_yellow = np.array([25, 70, 150])
    upper_yellow = np.array([40, 255, 255])
    
    blurred_image = cv2.GaussianBlur(img, (7, 7), 0)
    hsv = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    result = cv2.bitwise_and(img, img, mask=mask)

    gray_img = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

    masked = cv2.bitwise_and(gray_img, MASK)



    pixel_y, pixel_x = np.where(masked > 0)

    if len(pixel_x) < 10: # Not enough pixels to make a decision
        return 0
    
    # 1. Find continuous blobs (contours) of color
    contours, _ = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    left_x, left_y = [], []
    right_x, right_y = [], []

    for contour in contours:
        # 2. Filter out tiny speckles/noise
        if cv2.contourArea(contour) < 150:  # Tune this threshold as needed
            continue

        # 3. Find where the lane starts at the bottom of the screen
        # In OpenCV, a higher Y coordinate means lower on the screen
        bottom_most_point = contour[contour[:, :, 1].argmax()][0]
        start_x = bottom_most_point[0]

        # 4. Extract all exact pixels belonging to this specific contour
        contour_mask = np.zeros_like(masked)
        cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
        c_y, c_x = np.where(contour_mask > 0)

        # 5. Assign the ENTIRE contour to left or right based on its starting position
        if start_x < HALF_WIDTH:
            left_x.extend(c_x)
            left_y.extend(c_y)
        else:
            right_x.extend(c_x)
            right_y.extend(c_y)

   # 3. Fit Lines (fitting x as a function of y: x = my + b)
    left_fit = None
    right_fit = None
    
    if len(left_x) > 50:
        temp_fit = np.polyfit(left_y, left_x, 1)
        # Left line slope in this coordinate system (x=f(y)) should be negative
        if -1.5 < temp_fit[0] < 0.1: # Threshold to ensure it's not tilting the wrong way
            left_fit = temp_fit

    if len(right_x) > 50:
        temp_fit = np.polyfit(right_y, right_x, 1)
        # Right line slope in this coordinate system should be positive
        if 1.5 > temp_fit[0] > -0.1: 
            right_fit = temp_fit

    # 4. Calculate Midline with improved fallback
    y1 = IMG_HEIGHT
    y2 = mask_tip_y

    # Constants for recovery
    LANE_WIDTH_PX = 1000 # Adjust based on your camera calibration

    if left_fit is not None and right_fit is not None:
        # Scenario A: Both lines detected
        l_bottom = get_x(left_fit, y1)
        r_bottom = get_x(right_fit, y1)
        l_top = get_x(left_fit, y2)
        r_top = get_x(right_fit, y2)
        
        lane_center_bottom = (l_bottom + r_bottom) // 2
        lane_center_lookahead = (l_top + r_top) // 2
        
    elif left_fit is not None:
        # Scenario B: Only Left line - extrapolate right line using slope
        l_bottom = get_x(left_fit, y1)
        l_top = get_x(left_fit, y2)
        
        lane_center_bottom = l_bottom + (LANE_WIDTH_PX // 2)
        lane_center_lookahead = l_top + (LANE_WIDTH_PX // 2)
        
        # Virtual right line for drawing
        right_fit = [left_fit[0], left_fit[1] + LANE_WIDTH_PX]

    elif right_fit is not None:
        # Scenario C: Only Right line - extrapolate left line using slope
        r_bottom = get_x(right_fit, y1)
        r_top = get_x(right_fit, y2)
        
        lane_center_bottom = r_bottom - (LANE_WIDTH_PX // 2)
        lane_center_lookahead = r_top - (LANE_WIDTH_PX // 2)
        
        # Virtual left line for drawing
        left_fit = [right_fit[0], right_fit[1] - LANE_WIDTH_PX]
        
    else:
        # Scenario D: Nothing found - use drive straight
        steering = 0.0
        return 0

    # 5. Steering Logic (CTE)
    CAMERA_OFFSET_PX = 30  # tune: positive = camera is right of center
    car_center = HALF_WIDTH - CAMERA_OFFSET_PX  # true vehicle centerline

    cte_bottom = car_center - lane_center_bottom
    cte_lookahead = car_center - lane_center_lookahead


    # Anticipate the turn, but stay grounded in the current lane
    cte = (cte_lookahead * 0.50) + (cte_bottom * 0.50) 

    max_steer_threshold = int(HALF_WIDTH * .7)
    max_steer_amount = 80.0
    
    LOOKAHEAD_PX = mask_tip_y

    angle_rad = math.atan2(cte, LOOKAHEAD_PX)
    angle_deg = math.degrees(angle_rad)

    steering = np.clip(angle_deg, -max_steer_amount, max_steer_amount)
    

# ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
# 10.138.194.207

    # # 6. Fixed Line Drawing
    # line_img = np.zeros_like(img)

    # # Draw detected lanes
    # if left_fit is not None:
    #     cv2.line(line_img, (get_x(left_fit, y1), y1), (get_x(left_fit, y2), y2), (0, 255, 0), 5)
    # if right_fit is not None:
    #     cv2.line(line_img, (get_x(right_fit, y1), y1), (get_x(right_fit, y2), y2), (0, 0, 255), 5)

    # # Draw steering midline (Pink) and vehicle center (White)
    # cv2.line(line_img, (int(lane_center_bottom), y1), (int(lane_center_lookahead), y2), (255, 0, 255), 3)
    # cv2.line(line_img, (HALF_WIDTH, y1), (HALF_WIDTH, y2), (255, 255, 255), 1)

    # overlay = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)
    # cv2.imshow('Lane Tracking', overlay)
    # cv2.imshow('Yellow Only', masked)


    # cv2.waitKey(1)

    return 0

def main():
    global PREVIOUS_DECISION

    rclpy.init()
    node = rclpy.create_node('lane_follower')
    node.create_subscription(Image, IMAGE_TOPIC, camera_callback, 10)
    
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    auto_steering = node.create_publisher(Int64, "/auto_steering",1)

    FREQ = 10
    rate = node.create_rate(FREQ, node.get_clock())
    
    while rclpy.ok() and image is None:
        print("Not receiving image topic")
        rate.sleep()

    while rclpy.ok():
        # Process one image. The return value will be use for `something` later.
        ret = process_image(image)
        
        constant = .8
        exponential = steering * constant + (1 - constant) * PREVIOUS_DECISION
        PREVIOUS_DECISION = exponential

        # Helps keep the car straight. The car naturally leans left.
        exponential += 6
        print("Steering: ", -exponential)
        wrapped_steering = Int64()
        wrapped_steering.data = int (-exponential)

        auto_steering.publish(wrapped_steering)
        
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
