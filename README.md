# vsss_sim

A simple ROS 2 simulation built for the IEEE VSSS competition.

---
>## Docker support for development environment
>
>If you wish to set up a Docker container to develop or automate commands in isolated environments please refer to [docker](/docker/README.md)
>
---


# Repository Setup

###  Dependencies

First, make sure your system is up to date:

```bash
sudo apt update
sudo apt upgrade
```

Then, navigate to your ROS 2 workspace directory (replace your_ws with your actual workspace name):

```bash
cd ~/your_ws
```

Run the following to install all required dependencies:

``` bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

This reads the package.xml files in your workspace and installs any missing system dependencies.

### Build and Source

Inside your workspace root (~/your_ws), build the workspace:

```bash
colcon build --symlink-install
```

Then, source it:

```bash
source install/setup.bash
```


# Packages
## vsss_simulation

For this packages there are two main launch files
- Simulation package
>This launch file: can recive a parameter of team_side selecting what side the robot will attack
```bash
ros2 launch vsss_simulation vsss_simulation.launch.py team_side:=<true/false>
``` 
- In Real Life launch
>This launch file recives the same parameters team_side
```bash
ros2 launch vsss_simulation irl_play.launch.py team_side:=<true/false>
``` 
---
As for the debuging, there are also two scripts
- Vector Grapher
> File to display in rviz2 the vector field, the attacker will follow
```bash
ros2 run vsss_simulation VectorGrapher
``` 
- Robot Action Visualizer
> Displays an arrow indicating the final objective of the corresponfing robot
```bash
ros2 run vsss_simulation RobotActionVisualizer <robot_name>
``` 
