import cv2
import depthai as dai
import contextlib

import os
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import time 
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy


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



class MultiCamNode(Node):

    def __init__(self):
        super().__init__("multi_cam_node")
        self.device_info = dai.Device.getAllAvailableDevices()
        self.q_rgb_list = []
        self.num_devices = len(self.device_info)
        self.cam_publishers = []
        self.bridge = CvBridge()
        self.mxids = []
    
        self.declare_parameter("capture", False)
        self.declare_parameter("save_path", "./")
        self.declare_parameter("img_width", 1448)
        self.declare_parameter("img_height", 568)
            
        self.capture = self.get_parameter("capture").get_parameter_value().bool_value
        self.img_save_path = self.get_parameter("save_path").get_parameter_value().string_value
        self.img_width = self.get_parameter("img_width").get_parameter_value().integer_value
        self.img_height = self.get_parameter("img_height").get_parameter_value().integer_value
        
        self.get_logger().info("MultiCamNode: capture: {}".format(self.capture))
        
           
        self.camera_initialization(capture = self.capture, path = self.img_save_path) #FIXME: CHANGE HERE TO ENTER DEBUG MODE

        self.camera_qos = QoSProfile(
                reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_BEST_EFFORT,
                history = QoSHistoryPolicy.RMW_QOS_POLICY_KEEP_LAST,
                depth = 1
            )

    def getPipeline(self, preview_res = (1448, 568)): # default to livox 
        # Start defining a pipeline
        pipeline = dai.Pipeline()

        # Define a source - color camera
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        # For the demo, just set a larger RGB preview size for OAK-D
        cam_rgb.setPreviewSize(preview_res[0], preview_res[1]) # FIX ME, need to match what ever pipeline we are using
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)

        # Create output
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        return pipeline


    def camera_initialization(self, capture = False, path = "./"):
        
        if not capture:
            self.get_logger().info(f"{bcolors.OKGREEN}Initializing Normal Camera Node{bcolors.ENDC}")

        # https://docs.python.org/3/library/contextlib.html#contextlib.ExitStack
        with contextlib.ExitStack() as stack:
            device_infos = dai.Device.getAllAvailableDevices()
            if len(device_infos) == 0:
                self.get_logger().info(f"{bcolors.FAIL}No device found{bcolors.ENDC}")
                exit()
            else:
                 self.get_logger().info(f"{bcolors.OKGREEN}Found {len(device_infos)} Devices! {bcolors.ENDC}")
                
    
            for device_info in device_infos:
                openvino_version = dai.OpenVINO.Version.VERSION_2021_4
                usb2_mode = False
                device = stack.enter_context(dai.Device(openvino_version, device_info, usb2_mode))

                # Note: currently on POE, DeviceInfo.getMxId() and Device.getMxId() are different!
                mxid = device.getMxId()
                self.mxids.append(mxid)
                self.cam_publishers.append(self.create_publisher(Image, "cam" + mxid + "/image", 1))
        
                self.get_logger().info(f"{bcolors.OKCYAN}Found DeviceID: {mxid} {bcolors.ENDC}")

                # Get a customized pipeline based on identified device type
                pipeline = self.getPipeline(preview_res=(self.img_width, self.img_height))
                device.startPipeline(pipeline)
                time.sleep(0.1)


                # Output queue will be used to get the rgb frames from the output defined above
                q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                stream_name = "rgb-" + mxid + "-" + "OAK-D"
                self.q_rgb_list.append((q_rgb, stream_name))
                
                self.get_logger().info(f"{bcolors.OKGREEN}Device {mxid} initialized! {bcolors.ENDC}")
        

            if capture:
                    self.get_logger().info(f"{bcolors.OKGREEN}Entering Image Save Mode{bcolors.ENDC}")
                    self.get_logger().info(f"{bcolors.OKCYAN}Press s to save image to \"{path}\"{bcolors.ENDC}")
                    self.get_logger().info(f"{bcolors.OKCYAN}Press q to quit{bcolors.ENDC}")
                    
                    img_cnt = 0
                    
                    while rclpy.ok():
                        for q_rgb, stream_name in self.q_rgb_list:
                            in_rgb = q_rgb.tryGet()
                            if in_rgb is not None:
                                cv2.imshow(stream_name, in_rgb.getCvFrame())
                                key = cv2.waitKey(1) & 0xFF
                                if key == ord('s'):
                                    cv2.imwrite(os.path.join(path, str(img_cnt) + '.bmp'), in_rgb.getCvFrame())
                                    self.get_logger().info(f"{bcolors.OKGREEN}Saved image {img_cnt}! {bcolors.ENDC}")
                                    img_cnt += 1
                                elif key == ord('q'):
                                    self.get_logger().info(f"{bcolors.WARNING}{bcolors.BOLD}Quitting...{bcolors.ENDC}")
                                    exit("user quit")
                    
            
            else:
                while rclpy.ok():
                    for i, (q_rgb, _) in enumerate(self.q_rgb_list):
                        in_rgb = q_rgb.tryGet()
                        if in_rgb is not None:
                            img_msg = self.bridge.cv2_to_imgmsg(in_rgb.getCvFrame(), "bgr8")
                            img_msg.header.stamp = self.get_clock().now().to_msg()
                            self.cam_publishers[i].publish(img_msg)
                    time.sleep(1/30)
                    
          
        

def main():
    rclpy.init()
    cam_node = MultiCamNode()
    rclpy.spin(cam_node)


if __name__ == "main":
    main()