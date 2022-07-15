import cv2
import depthai as dai
import message_filters
import os
import torch
import numpy as np
from multi_cam_obj_detection.multi_cam_obj_detection.utils.torch_utils import select_device
from utils.torch_utils import *

from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

class MultiCamSubscriber(Node):
        def __init__(self, queue = 1, slop = 0.1):
            super().__init__("multi_cam_sub_node")
            
            self.declare_parameter("model_path", "")
            self.declare_parameter("confidence_threshold", 0.5)
            self.declare_parameter("queue_size", 1)
            self.declare_parameter("slop", 0.1)
            
            
            self.num_cams = len(dai.Device.getAllAvailableDevices())
            self._num_cams = 0
            
            # In case that getAllAvailableDevices() returns empty list
            _cam = Node("cam_sub")
            _cam.create_subscription(Int16, "device/info", self.device_info_callback, 1)
            rclpy.spin_once(_cam, timeout_sec=1)
            
            
            if not self.num_cams and not self._num_cams:
                exit("No device found, please connect a device and try again.")
                
            self.sync_queue = self.get_parameter('queue_size').get_parameter_value().integer_value
            self.sync_slop = self.get_parameter('slop').get_parameter_value().double_value
            self.camera_subs = []
            self.bridge = CvBridge()
            
            
            self.device = select_device()
            
            for i in range(self.num_cams):
                self.camera_subs.append(message_filters.Subscriber(self, Image, "camera/image_" + str(i)))
            
            ts = message_filters.ApproximateTimeSynchronizer(self.camera_subs, self.sync_queue, self.sync_slop)
            ts.registerCallback(self.on_images_recieve)
              
        
        def device_info_callback(self, msg):
            self._num_cams = msg.data
            
        
        def on_images_recieve(self, *args):
            images = [self.bridge.imgmsg_to_cv2(msg, "rgb8") for msg in args] # RGB, ndarray
            self.process_images(images)
            
        def process_images(self, images):
            image_batch = np.zeros((len(images), images[0].shape[0], images[0].shape[1], 3))
            for i in range(len(images)):
                image_batch[i] = images[i]
            
            image_tensor = torch.from_numpy(image_batch).to_device(self.device)
            
        

def main():
    rclpy.init()
 
    cam_sub = MultiCamSubscriber()

    rclpy.spin(cam_sub)

if __name__ == "__main__":
    main()