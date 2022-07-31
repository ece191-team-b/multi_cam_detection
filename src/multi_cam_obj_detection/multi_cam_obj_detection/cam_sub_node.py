import cv2
import depthai as dai
import message_filters
import os
import torch
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from rclpy.parameter import Parameter
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

class MultiCamSubscriber(Node):
        def __init__(self):
            super().__init__("multi_cam_sub_node")
            _cam = Node("cam_subs")
            
            self.get_logger().info("Initializing MultiCamSubscriber")
            
            # Declare parameters
            self.declare_parameter("model_path", "")
            self.declare_parameter("confidence_threshold", 0.5)
            self.declare_parameter("queue_size", 1)
            self.declare_parameter("slop", 0.1)
            self.declare_parameter("cam_topics", ["None"]) 
            
            
            # Get parameters
            model_path = self.get_parameter('model_path').get_parameter_value().string_value
            self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
            self.sync_queue = self.get_parameter('queue_size').get_parameter_value().integer_value
            self.sync_slop = self.get_parameter('slop').get_parameter_value().double_value
            self.topics = self.get_parameter('cam_topics').get_parameter_value().string_array_value
            
            self.get_logger().info("Loading model from {}".format(model_path))
            
            self.model = torch.hub.load(model_path, 'yolov5s', source="local")
            
            self.get_logger().info("self.topics: %s", self.topics)
             
            self.camera_subs = []
            self.bridge = CvBridge()
            
            self.num_cams = max(self.num_cams, self._num_cams)
            
            for topic in self.topics:
                self.camera_subs.append(message_filters.Subscriber(self, Image, topic))
            
            ts = message_filters.ApproximateTimeSynchronizer(self.camera_subs, self.sync_queue, self.sync_slop)
            ts.registerCallback(self.on_images_recieve)
              
        
        def device_info_callback(self, msg):
            self._num_cams = msg.data
            
        
        def on_images_recieve(self, *args):
            images = [self.bridge.imgmsg_to_cv2(msg, "rgb8") for msg in args] # RGB, ndarray
            # dim = (640, 640)
            # images = [cv2.resize(image, dim, interpolation=cv2.INTER_AREA) for image in images]
            # self.process_images(images)
            print(len(images))
            
            
        def process_images(self, images):
            
            output = self.model(images)
            
        

def main():
    rclpy.init()
 
    cam_sub = MultiCamSubscriber()

    rclpy.spin(cam_sub)

if __name__ == "__main__":
    main()