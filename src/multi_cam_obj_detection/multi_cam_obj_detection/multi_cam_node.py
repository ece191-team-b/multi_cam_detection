import cv2
import depthai as dai
import contextlib

from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node


class MultiCamNode(Node):

    def __init__(self):
        super().__init__("multi_cam_node")
        self.device_info = dai.Device.getAllAvailableDevices()
        self.q_rgb_list = []
        self.num_devices = len(self.device_info)
        self.cam_publishers = []
        self.bridge = CvBridge()
        self.device_publisher = self.create_publisher(Int16, "device/info", 10)
        self.mxids = []
        


    def getPipeline(self, preview_res = (600, 300)):
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


    def camera_initialization(self, debug = False):

        # https://docs.python.org/3/library/contextlib.html#contextlib.ExitStack
        with contextlib.ExitStack() as stack:
            device_infos = dai.Device.getAllAvailableDevices()
            if len(device_infos) == 0:
                print("No device found")
                exit()
            else:
                print("Found", len(device_infos), "devices")

            for device_info in device_infos:
                openvino_version = dai.OpenVINO.Version.VERSION_2021_4
                usb2_mode = False
                device = stack.enter_context(dai.Device(openvino_version, device_info, usb2_mode))

                # Note: currently on POE, DeviceInfo.getMxId() and Device.getMxId() are different!
                print("=== Connected to " + device_info.getMxId())
                mxid = device.getMxId()
                self.mxids.append(mxid)
                self.cam_publishers.append(self.create_publisher(Image, "cam" + mxid + "/image", 10))
                print("   >>> MXID:", mxid)

                # Get a customized pipeline based on identified device type
                pipeline = self.getPipeline()
                print("   >>> Loading pipeline for: OAK-D-LITE")
                device.startPipeline(pipeline)

                # Output queue will be used to get the rgb frames from the output defined above
                q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                stream_name = "rgb-" + mxid + "-" + "OAK-D"
                self.q_rgb_list.append((q_rgb, stream_name))

            if debug:
                    self.image_display_opencv()
                
            else:
                while rclpy.ok():
                    num_device_msg = Int16()
                    num_device_msg.data = self.num_devices
                    self.device_publisher.publish(num_device_msg)
                    for i, (q_rgb, _) in enumerate(self.q_rgb_list):
                        in_rgb = q_rgb.tryGet()
                        if in_rgb is not None:
                            img_msg = self.bridge.cv2_to_imgmsg(in_rgb.getCvFrame(), "bgr8")
                            self.cam_publishers[i].publish(img_msg)


def main():
    rclpy.init()
    cam_node = MultiCamNode()
    cam_node.camera_initialization()


if __name__ == "main":
    main()