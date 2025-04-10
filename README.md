# vsss_sim
Small ROS2 simulation made for IEEE VSSS competition

## How to run
This launch instruction will launch the Gazebo simulation of the robot, including a top down camera and the field
```python3
ros2 launch vsss_simulation simulation_spawn.launch.py 

```

## How to see what the camera's POV

```python3
ros2 run rqt_image_view rqt_image_view
```

Feel free to modify this package according to your needs
