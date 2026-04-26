#!/usr/bin/env python

import math
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int64

# Constants
IMAGE_TOPIC = "/zed/zed_node/right/color/rect/image"
STEER_TOPIC = "/auto_steering"

FREQ = 40

HEIGHT     = int(540 / 2)
WIDTH      = int(960 / 2)
IMG_HEIGHT = HEIGHT
IMG_WIDTH  = WIDTH
HALF_WIDTH = IMG_WIDTH // 2

LANE_WIDTH_PX = 500

CAR_CAMERA_OFFSET  = -15
LOOKAHEAD_WEIGHT   = 1
MAX_STEER_SCALER   = 60
MAX_STEER_AMOUNT   = 60

CONTOUR_AREA_MIN   = 50
CONTOUR_POINTS_MIN = 20
EDGE_MARGIN        = 5

# Yellow lane colour ranges (HSV)
LOWER_YELLOW      = np.array([25, 100, 150])
UPPER_YELLOW      = np.array([40, 255, 255])
LOWER_PALE_YELLOW = np.array([20,  60, 200])
UPPER_PALE_YELLOW = np.array([45, 100, 255])

# ROI mask
MASK         = np.zeros((IMG_HEIGHT, IMG_WIDTH), dtype=np.uint8)
MASK_EDGE    = IMG_HEIGHT - IMG_HEIGHT // 3
MASK_TIP_X   = IMG_WIDTH  // 2
MASK_TIP_Y   = IMG_HEIGHT // 2 + IMG_HEIGHT // 12

vertices = [np.array([
    [0,         IMG_HEIGHT],
    [0,         MASK_EDGE ],
    [MASK_TIP_X, MASK_TIP_Y],
    [IMG_WIDTH,  MASK_EDGE ],
    [IMG_WIDTH,  IMG_HEIGHT],
], dtype=np.int32)]
cv2.fillPoly(MASK, vertices, (255, 255, 255))

# State
br                 = CvBridge()
image              = None
steering           = 0
PREVIOUS_DECISIONS = [0, 0]
OLDEST_DECISION    = 0

# Callbacks
def camera_callback(data):
    global image
    image = br.imgmsg_to_cv2(data)
    image = image[:, :, :3]


def get_x(fit, y) -> int:
    """Return the x pixel coordinate on a fitted line at height y."""
    return int(fit[0] * y + fit[1])


def drawline(img, color, line):
    LINE_THICKNESS = 3
    DOT_SIZE       = 3
    for x1, y1, x2, y2 in line:
        cv2.line(img, (x1, y1), (x2, y2), color, LINE_THICKNESS)
        cv2.circle(img, (x1, y1), DOT_SIZE, color, -1)
        cv2.circle(img, (x2, y2), DOT_SIZE, color, -1)


def repack(line_list):
    x_list, y_list = [], []
    for line in line_list:
        for x1, y1, x2, y2 in line:
            x_list += [x1, x2]
            y_list += [y1, y2]
    return x_list, y_list


def process_image(img):
    global steering, PREVIOUS_DECISIONS, OLDEST_DECISION

    img = cv2.resize(img, (WIDTH, HEIGHT))

    hsv          = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    yellow_mask  = cv2.inRange(hsv, LOWER_YELLOW,      UPPER_YELLOW)
    pale_mask    = cv2.inRange(hsv, LOWER_PALE_YELLOW, UPPER_PALE_YELLOW)
    combined     = cv2.bitwise_or(yellow_mask, pale_mask)
    masked       = cv2.bitwise_and(combined, MASK)


    contours, _ = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    candidate_lines = []
    for contour in contours:
        if cv2.contourArea(contour) < CONTOUR_AREA_MIN:
            continue

        contour_mask = np.zeros_like(masked)
        cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)
        c_y, c_x = np.where(contour_mask > 0)

        if len(c_x) < CONTOUR_POINTS_MIN:
            continue

        # Only keep blobs that touch an image edge
        touches_bottom = np.any(c_y >= IMG_HEIGHT - EDGE_MARGIN)
        touches_left   = np.any(c_x <= EDGE_MARGIN)
        touches_right  = np.any(c_x >= IMG_WIDTH  - EDGE_MARGIN)
        touches_top    = np.any(c_y <= EDGE_MARGIN)
        if not (touches_bottom or touches_left or touches_right or touches_top):
            continue

        fit          = np.polyfit(c_y, c_x, 1)
        x_at_bottom  = int(fit[0] * IMG_HEIGHT + fit[1])
        candidate_lines.append({"fit": fit, "x_at_bottom": x_at_bottom})

    # Sort left to right and assign lanes
    candidate_lines.sort(key=lambda l: l["x_at_bottom"])

    left_fit  = None
    right_fit = None

    if len(candidate_lines) == 2:
        left_fit  = candidate_lines[0]["fit"]
        right_fit = candidate_lines[1]["fit"]
    elif len(candidate_lines) == 1:
        line = candidate_lines[0]
        if line["x_at_bottom"] <= HALF_WIDTH:
            left_fit  = line["fit"]
        else:
            right_fit = line["fit"]

    # lane center calculations
    y_bottom = IMG_HEIGHT
    y_top    = MASK_TIP_Y

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


    cte = (
        HALF_WIDTH
        - lane_center_lookahead * LOOKAHEAD_WEIGHT
        - lane_center_bottom    * (1 - LOOKAHEAD_WEIGHT)
        + CAR_CAMERA_OFFSET
    )
    print("cte:", cte)

    if math.fabs(cte) > 0:
        steering = math.copysign(cte / HALF_WIDTH * MAX_STEER_SCALER, cte)
        if math.fabs(steering) > MAX_STEER_AMOUNT:
            steering = math.copysign(MAX_STEER_AMOUNT, cte)
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
    rclpy.init()
    node = rclpy.create_node("lane_follower")
    node.create_subscription(Image, IMAGE_TOPIC, camera_callback, 10)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    auto_steering_pub = node.create_publisher(Int64, STEER_TOPIC, 1)

    rate = node.create_rate(FREQ, node.get_clock())

    while rclpy.ok() and image is None:
        print("Not receiving image topic")
        rate.sleep()

    while rclpy.ok():
        process_image(image)

        average_steering      = sum(PREVIOUS_DECISIONS) / len(PREVIOUS_DECISIONS)
        print("Steering: ", average_steering)

        wrapped_steering      = Int64()
        wrapped_steering.data = int(average_steering)
        auto_steering_pub.publish(wrapped_steering)

        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()