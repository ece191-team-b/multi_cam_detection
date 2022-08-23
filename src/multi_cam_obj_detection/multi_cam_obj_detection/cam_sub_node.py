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


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class MultiCamSubscriber(Node):
        def __init__(self):
            super().__init__("cam_subs")
            # _cam = Node("cam_subs")
            
            self.get_logger().info(f"{bcolors.OKGREEN}Initializing MultiCamSubscriber{bcolors.ENDC}")
            
            # Declare parameters
            self.declare_parameter("model_path", "./model")
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
            
            self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
            
            self.get_logger().info("self.topics: " + str(self.topics))
             
            self.camera_subs = []
            self.bridge = CvBridge()
            
            # self.num_cams = max(self.num_cams, self._num_cams)
            
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
            print("Received images")
            self.process_images(images)
            print(len(images))
            
            
        def process_images(self, images):
            pred = self.model(images)
            print(type(pred))
            print(pred)
            exit()
        

def main():
    rclpy.init()
 
    cam_sub = MultiCamSubscriber()

    rclpy.spin(cam_sub)

if __name__ == "__main__":
    main()