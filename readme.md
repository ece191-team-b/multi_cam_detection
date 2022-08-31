# Multi Cam Object Detection 
## Installation 
Please first install ros2 galactic. Then install required ROS2 packages.
```bash
sudo apt install ros-galactic-vision-msg
sudo apt install ros-galactic-cv-bridge
pip install -r requirements.txt 
pip install depthai
pip install opencv-python
```
After that, please build the package with
```bash
colcon build --packages-select multi_cam_obj_detection
```
and source with 
```bash
source install/setup.bash
```
to rebuld the package from scratch please run
```
rm -rf install
rm -rf build
colcon build
```
**Note: remove install/ and build/ will remove all built packages**


## Usage
### Calibration
During Calibration, Please only connect **one camera** at a time.
1. Please go to **_src/multi_cam_obj_detection/config/img_capture.yaml_** to change the path the iamge will save to. If you want to change the resolution, it is where you will change as well. **Note: The maximum resolution is set to 1080P, that is the resolution you pick can not go beyond that for now, if you wish to go up to 4k, please change multi_cam_node.py accordingly.** **Use _ABSOLUTE PATH_ to save iamge.**
2. Run
```bash
ros2 launch multi_cam_obj_detection calibration_image_capture.launch.py
```
After that a window will pop up showing the image, please click on that window and follow instruction prompted in terminal to capture image, the default format is bmp. 
3. **Note that at this time the camera's MXID will be printed to the terminal, please record that as it will be in later use.**

### Object Detection and BBOX Publishing
Before starting the whole pipeline, please go to **_src/multi_cam_obj_detection/launch/cam_launch.py_** to remap the topics with camera's MXID and to the topic of your preference. **NOTE: If you are only using one camera, do not add more than one topic as message_filter will block callback until msg recieved from that topic.**
1.  Please then go to **_src/multi_cam_obj_detection/config/cam_sub.yaml_** and add the remapped to topic to _topics_ as a **list**. Please also change other parameters accordingly. Please also specify the topics where bounding box is published to. **NOTE: All paths should be ABSOLUTE PATH** 
2.  Launch cameras with
```bash
ros2 launch multi_cam_obj_detection cam_launch.launch.py
```
Make sure the camera is running, run 
```bash
rviz2
```
and click on add, by topics and select the topics you set to see if images present. 
3. Launch detection node
```bash
ros2 launch multi_cam_obj_detection sub_launch.launch.py
```
***DOCUMENTATION STILL ONGOING***

