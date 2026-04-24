#!/usr/bin/env python3
"""
Lab Starter: YOLO Object Detection with ROS2
--------------------------------------------

Subscribes : /zed/zed_node/rgb/color/rect/image  (sensor_msgs/Image)
Publishes  : /detections/image             (sensor_msgs/Image)  -- annotated image
             /detections/count             (std_msgs/Int32)     -- number of objects
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Int32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

HEIGHT = int(540/2)
WIDTH = int(960/2)

class YoloDetectionNode(Node):

    def __init__(self):
        super().__init__('yolo_detection_node')

        # Model
        self.model = YOLO('yolov8n.pt')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, '/zed/zed_node/right/color/rect/image', self.image_callback, 1)

        self.pub_image = self.create_publisher(Image, '/detections/image', 1)
        self.pub_count = self.create_publisher(Int32, '/detections/count', 1)

        # TODO: Create topic to send "STOP" Signalwe
        ###############################
        ## Your code here
        ###############################
        self.stop_pub = self.create_publisher(Int32MultiArray, '/detections/stop', 1)


        self.get_logger().info('YOLO detection node ready')

    # Callback
    def image_callback(self, msg: Image):
        # Convert ROS Image to OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        print(frame.shape)

        # Run inference
        ## TA MOD: USING ORIGINAL FRAME FOR TESTING PURPOSES
        results = self.model(frame)
        annotated = results[0].plot()          # draws boxes on the frame
        num_objects = len(results[0].boxes)

        
        # TODO: Publish annotated image
        ###############################
        ## Your code here
        ###############################
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))
        #cv2.imshow("test", annotated)
        #cv2.waitKey(1) 

        # TODO: Publish object count
        ###############################
        ## Your code here
        ###############################
        wrapped_int = Int32()
        wrapped_int.data = int (num_objects)
        self.pub_count.publish(wrapped_int)

        # TODO: Get "Stop Sign" Size
        # if area >= 300 * 300 
        # STOP
        ###############################
        ## Your code here
        ############################### 

        stop_signal = 0
        final_area = 0


        wrapped_multi = Int32MultiArray()
        
        for index,id in enumerate(results[0].boxes.cls):
            if (id == 11):
                for x1, y1, x2, y2 in results[0].boxes[index].xyxy:

                    x = x2 - x1
                    y = y2 - y1

                    print(x, y)

                    final_area = int(y)

                    if final_area >= 50:
                        stop_signal = 1

        wrapped_multi.data = [stop_signal, final_area]
        print(wrapped_multi)
        self.stop_pub.publish(wrapped_multi)
                

       

        pass  # remove once TODOs are filled in


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

