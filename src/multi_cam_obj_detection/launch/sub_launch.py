from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
     
     config = os.path.join(get_package_share_directory('multi_cam_obj_detection'),
        'config',
        'multi_cam_topic.yaml'
        )
     
     return LaunchDescription([
      Node(package='multi_cam_obj_detection',
             name = "subs",
             executable='subs',
             parameters = [config]),
         
    ]) 