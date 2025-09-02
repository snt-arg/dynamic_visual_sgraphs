# 📹 Using RealSense Cameras in vS-Graphs

![RealSense](realsense.png "RealSense")

If you want to use **RealSense** cameras to collect data or run vS-Graphs in the live mode, take a look at the notes below (valid for **RealSense D400 Series**):

## ⚙️ Installation

The fastest solution is to use `realsense-ros` wrapper, available in [this link](https://github.com/IntelRealSense/realsense-ros/tree/ros2-master).
A quick guide is also available below. Please note that this guide is prepared based on **ROS2 Jazzy** version of the library, compatible with `vS-Graphs`:

```bash
# Catkin workspace directory
mkdir -p ~/workspace/src
cd ~/workspace/src/

# Install required libraries
sudo apt-get update && apt-get install -y \
    ros-jazzy-rviz-visual-tools \
    ros-jazzy-depth-image-proc \
    ros-jazzy-backward-ros \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-pcl-ros

# Clone the repository
git clone -b ros2-master git@github.com:IntelRealSense/realsense-ros.git

# Build the workspace
cd ~/workspace/
source /opt/ros/jazzy/setup.bash && rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Run a simple test
source /opt/ros/jazzy/setup.bash && source ~/workspace/install/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

## 📝 Launch File Modification

You can also modify the launch file provided for launching **RealSense** to capture proper topics required in `vS-Graphs`. A sample of such file is available [here](https://github.com/IntelRealSense/realsense-ros/blob/ros2-master/realsense2_camera/launch/rs_launch.py).

Alternatively, you can directly fuse proper topics while running the **RealSense**, as below:

```bash
ros2 launch realsense2_camera rs_launch.py \
  pointcloud.enable:=true \
  align_depth.enable:=true \
  rgb_camera.color_profile:="640,480,30" \
  depth_camera.depth_profile:="640,480,30" \
  enable_accel:=true \
  enable_gyro:=true \
  gyro_fps:=200 \
  accel_fps:=63 \
  unite_imu_method:=0
```

### ⚠️ Using Presets

**RealSense D400** series contain various presets, well-adapted for different applications, such as hand gesture recognition or robotics.
The argument `json_file_path` in the launch files described above keeps the path to these presets (if not provided, it uses the default preset).
In vS-Graphs, [high-accuracy](/doc/RealSense/presets/HighAccuracyPreset.json) preset can provide high confidence threshold value of depth and lower fill factor.
You can read more about the available presets in [this link](https://dev.intelrealsense.com/docs/d400-series-visual-presets).

Accordingly, you can download a preset (also available [here](/doc/RealSense/presets/)) and set the link to the preset file as the `json_file_path` argument.

```bash
# (Option 1) Inside the launch file
{
  'name': 'json_file_path',
  'default': "YOUR_DOWNLOADED_FILE_PATH", # e.g., ~/HighAccuracyPreset.json
  'description': 'allows advanced configuration'
}

# (Option 2) Through arguments
ros2 launch realsense2_camera rs_launch.py json_file_path:='YOUR_DOWNLOADED_FILE_PATH'
```

## 🔨 Calibration

Generally, no calibration process is required for **D400** seriers RGB cameras.

### Calibrating IMU

In case you need to use the **IMU sensor**, you first need to calibrate it according to the guideline provided [here](/doc/RealSense/calibration/rs_d435i_imu_calibration.pdf). Accordingly, you need to run the [Python Script](/doc/RealSense/calibration/rs-imu-calibration.py), follow the steps shown in the terminal, and save the calibration parameters **into the camera**.

## 💽 Data Collection

The type of data fed to vS-Graphs is either a `ros2 bag` file or live camera feed. For recording a `ros2 bag` using a **RealSense D400** series camera, follow the steps provided below:

1. Make sure you have installed the required libraries for **RealSense** and **ROS2 Jazzy**.
2. Run the  using `ros2 launch realsense2_camera rs_launch.launch [params]`.
3. Navigate to the directory where you plan to save the `ros2 bag` files.
4. Finally, run the below and record the topics of interest:

```bash
ros2 bag record /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /camera/color/camera_info /camera/aligned_depth_to_color/camera_info /camera/imu
```
