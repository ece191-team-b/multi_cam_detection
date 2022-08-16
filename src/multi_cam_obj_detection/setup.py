from setuptools import setup
import os
from glob import glob

package_name = 'multi_cam_obj_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chengjing',
    maintainer_email='cyuan@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cams = multi_cam_obj_detection.multi_cam_node:main',
            'subs = multi_cam_obj_detection.cam_sub_node:main',
            'imgs = multi_cam_obj_detection.calibration_img_capture_node:main',
        ],
    },
)
