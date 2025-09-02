from setuptools import find_packages, setup

package_name = 'vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vision_launch.launch.py']),
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
            'model_use = vision.vision_general:main',
            'ball_detect = vision.camera_ball:main'
        ],
    },
)
