# vsss_sim

A simple ROS 2 simulation built for the IEEE VSSS competition.

---
## 📦 Dependencies

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

## 🔨 Build and Source

Inside your workspace root (~/your_ws), build the workspace:

```bash
colcon build --symlink-install
```

Then, source it:

```bash
source install/setup.bash
```

## 🧭 Launch Simulation
To start the Gazebo simulation (robot + field + overhead camera), run:

```bash
ros2 launch vsss_simulation simulation_spawn.launch.py 

```

## 📷 View the Camera Feed
To view the camera's point of view, use rqt_image_view:

```python3
ros2 run rqt_image_view rqt_image_view
```

Or you can use the topi **`/camera/image_raw`**

## 🛠️ Notes
Feel free to modify or extend this package to fit your specific simulation needs.
