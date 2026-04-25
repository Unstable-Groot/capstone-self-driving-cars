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
from cv_bridge import CvBridge

#Default: stereo image (from both L and R lenses)
IMAGE_TOPIC = '/zed/zed_node/right/color/rect/image'
STEER_TOPIC = "/auto_steering"

steering = 0
HEIGHT = int(540/2)
WIDTH = int(960/2)

IMG_HEIGHT = HEIGHT
IMG_WIDTH = int(WIDTH)

HALF_WIDTH = int(IMG_WIDTH / 2)


MASK = np.zeros((IMG_HEIGHT, IMG_WIDTH), dtype=np.uint8)
mask_edge = IMG_HEIGHT - int(IMG_HEIGHT/3)
mask_tip_x = int(IMG_WIDTH/2)
mask_tip_y = int(IMG_HEIGHT/2) + int(IMG_HEIGHT/12)

PREVIOUS_DECISIONS = [0, 0]
OLDEST_DECISION = 0

lower_yellow = np.array([25, 100, 150])
upper_yellow = np.array([40, 255, 255])

lower_pale_yellow = np.array([20, 60, 200]) 
upper_pale_yellow = np.array([45, 100, 255])

# Constants for recovery
LANE_WIDTH_PX = 500 # Adjust based on your camera calibration

vertices = [ np.array([ [0,IMG_HEIGHT], [0,mask_edge], [mask_tip_x,mask_tip_y], [IMG_WIDTH,mask_edge], [IMG_WIDTH,IMG_HEIGHT] ], dtype=np.int32) ]
cv2.fillPoly(MASK, vertices, (255,255,255))

camera = None
publish_image = None

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
    global publish_image
    ######
    # YOUR CODE: Lane Detection
    ######

    #Example: If you want to use a smaller image
    img = cv2.resize(img, (WIDTH, HEIGHT))
    # cv2.imshow('Small', img)


    # Yellow pass filter
    
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    pale_mask = cv2.inRange(hsv, lower_pale_yellow, upper_pale_yellow)

    combined_mask = cv2.bitwise_or(yellow_mask, pale_mask)

    masked = cv2.bitwise_and(combined_mask, MASK)

    

    
    # 1. Find continuous blobs (contours) of color
    contours, _ = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    candidate_lines = []

    for contour in contours:
        if cv2.contourArea(contour) < 50:
            continue

        contour_mask = np.zeros_like(masked)
        cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
        c_y, c_x = np.where(contour_mask > 0)

        if len(c_x) < 20:
            continue

        # Only keep blobs that touch an edge
        touches_bottom = np.any(c_y >= IMG_HEIGHT - 5)
        touches_left   = np.any(c_x <= 5)
        touches_right  = np.any(c_x >= IMG_WIDTH - 5)
        touches_top    = np.any(c_y <= 5)

        if not (touches_bottom or touches_left or touches_right or touches_top):
            continue

        contour_fit = np.polyfit(c_y, c_x, 1)
        x_at_bottom = int(contour_fit[0] * IMG_HEIGHT + contour_fit[1])

        candidate_lines.append({
            'fit': contour_fit,
            'x_at_bottom': x_at_bottom,
        })

    # Sort left to right, then just assign by position
    candidate_lines.sort(key=lambda l: l['x_at_bottom'])

    left_fit = None
    right_fit = None

    if len(candidate_lines) == 2:
        left_fit  = candidate_lines[0]['fit']
        right_fit = candidate_lines[1]['fit']
    elif len(candidate_lines) == 1:
        line = candidate_lines[0]
        if line['x_at_bottom'] <= HALF_WIDTH:
            left_fit = line['fit']
        else:
            right_fit = line['fit']
    # else: 0 lines, both remain None -> hits Scenario D

    # 4. Calculate Midline with improved fallback
    y1 = IMG_HEIGHT
    y2 = mask_tip_y

    print(left_fit, right_fit)



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
        lane_center_lookahead = l_top + (LANE_WIDTH_PX // 6)
        
        # Virtual right line for drawing
        right_fit = [left_fit[0], left_fit[1] + LANE_WIDTH_PX]

    elif right_fit is not None:
        # Scenario C: Only Right line - extrapolate left line using slope
        r_bottom = get_x(right_fit, y1)
        r_top = get_x(right_fit, y2)
        
        lane_center_bottom = r_bottom - (LANE_WIDTH_PX // 2)
        lane_center_lookahead = r_top - (LANE_WIDTH_PX // 6)
        
        # Virtual left line for drawing
        left_fit = [right_fit[0], right_fit[1] - LANE_WIDTH_PX]
        
    else:
        # Scenario D: Nothing found - use drive straight
        steering = 0.0
        return 0


    car_camera_offset = -15

    lookahead_weight = 1
    cte = HALF_WIDTH - lane_center_lookahead * lookahead_weight - lane_center_bottom * (1 - lookahead_weight) + car_camera_offset
    print("cte: ",cte)
    #set steering amount
    max_steer_threshold = HALF_WIDTH 
    steering_scaler = 60
    max_steer_amount = 60
    if(math.fabs(cte) > 0):
        steering = math.copysign((cte / max_steer_threshold * steering_scaler), cte)
        

        if (math.fabs(steering) > max_steer_amount):
            steering = math.copysign(max_steer_amount, cte)

        steering *= -1

    PREVIOUS_DECISIONS[OLDEST_DECISION] = steering
    OLDEST_DECISION = (OLDEST_DECISION + 1) % len(PREVIOUS_DECISIONS)
    

# ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
# 10.138.194.207

#     # 6. Fixed Line Drawing
#     line_img = np.zeros_like(img)

#     # Draw detected lanes
#     if left_fit is not None:
#         cv2.line(line_img, (get_x(left_fit, y1), y1), (get_x(left_fit, y2), y2), (0, 255, 0), 5)
#     if right_fit is not None:
#         cv2.line(line_img, (get_x(right_fit, y1), y1), (get_x(right_fit, y2), y2), (0, 0, 255), 5)

#     # Draw steering midline (Pink) and vehicle center (White)
#     cv2.line(line_img, (int(lane_center_bottom), y1), (int(lane_center_lookahead), y2), (255, 0, 255), 3)
#     cv2.line(line_img, (HALF_WIDTH, y1), (HALF_WIDTH, y2), (255, 255, 255), 1)

#     overlay = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)

#    # Create an empty 3-channel image for the blue mask
#     blue_mask_visual = np.zeros_like(img)
#     # Set the Blue channel (index 0 in BGR) to the values of 'masked'
#     blue_mask_visual[:, :, 0] = masked 

#     # Combine the blue mask with the previous overlay
#     overlay2 = cv2.addWeighted(overlay, 0.8, blue_mask_visual, 1.0, 0.0)

#     #cv2.imshow('Lane Tracking', overlay)
#     #cv2.imshow('Yellow Only', masked)
    

#     publish_image = overlay2

   # cv2.waitKey(1)

    return 0

def main():
    global camera, publish_image

    rclpy.init()
    node = rclpy.create_node('lane_follower')
    node.create_subscription(Image, IMAGE_TOPIC, camera_callback, 10)
    
    # bridge = CvBridge()

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    auto_steering = node.create_publisher(Int64, "/auto_steering",1)
    # camera = node.create_publisher(Image, "/custom_camera",1)

    FREQ = 40
    rate = node.create_rate(FREQ, node.get_clock())
    
    while rclpy.ok() and image is None:
        print("Not receiving image topic")
        rate.sleep()

    while rclpy.ok():
        # Process one image. The return value will be use for `something` later.
        ret = process_image(image)
        average_steering = sum(PREVIOUS_DECISIONS) / len(PREVIOUS_DECISIONS)
        print("Steering: ", average_steering)
        wrapped_steering = Int64()
        wrapped_steering.data = int (average_steering)

        auto_steering.publish(wrapped_steering)

        # if publish_image is not None:
        #     print("publishing")
        #     camera.publish(bridge.cv2_to_imgmsg(publish_image, 'bgr8'))
        # else:
        #     print("why not")
        
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
