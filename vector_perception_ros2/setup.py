from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vector_perception_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('lib', package_name),
         [os.path.join(package_name, 'checkpoint_detection.tar'),
          os.path.join(package_name, 'lib_cxx.so'),
          os.path.join(package_name, 'gsnet.so')]),
        (os.path.join('lib', package_name, 'license'),
         glob(os.path.join(package_name, 'license', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alexlin',
    maintainer_email='alex.lin416@outlook.com',
    description='ROS2 interface for Vector Perception segmentation and 3D perception',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'segment_3d_node = vector_perception_ros2.segment_3d_node:main',
            'grasp_node = vector_perception_ros2.grasp_node:main',
        ],
    },
)