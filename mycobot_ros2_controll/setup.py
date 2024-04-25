from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'mycobot_ros2_controll'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, "config"), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kairos',
    maintainer_email='mango7460@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rviz_controll=mycobot_ros2_controll.rviz_controll:main',
            'cam_pub=mycobot_ros2_controll.camera_pub:main',
            'detect_cube_hsv=mycobot_ros2_controll.detect_cube_hsv:main',
            'detect_cube_server=mycobot_ros2_controll.detect_cube_server:main'
        ],
    },
)
