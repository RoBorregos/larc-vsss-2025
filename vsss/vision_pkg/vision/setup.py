from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/utils/LUTs', glob('utils/LUTs/*.npy')),
        ('share/' + package_name + '/utils', glob('utils/*.npy')),
        ('share/' + package_name + '/utils/models/yolov8m(v1)', ['utils/models/yolov8m(v1)/best.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dany',
    maintainer_email='A00838702@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_input = vision.camera_input:main',
            'image_warp = vision.imageWarp:main',
            'vision_general = vision.vision_general:main',
            'ball_detect = vision.camera_ball:main'
        ],
    },
)
