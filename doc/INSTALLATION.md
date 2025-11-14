# 🚀 Install vS-Graphs

## 📝 I. Package Installation

### Install OpenCV

vS-Graphs requires **OpenCV v4.2+** for computer vision tasks, which can be installed via the [installation instructions](https://docs.opencv.org/4.x/d0/d3d/tutorial_general_install.html) page. For Ubuntu 24.04, install it using below commands.

```bash
sudo apt update
sudo apt install libopencv-dev -y

# Check the version
dpkg -l libopencv-dev
```

### Install Eigen3

vS-Graphs relies on **Eigen3**, a C++ template library for linear algebra (matrices, vectors, solvers, etc.). You can install it via APT:

```bash
sudo apt install libeigen3-dev
```

> 🛎️ Tip: `libeigen3-dev` is a header-only library and no linking is required, as `CMake` will automatically find it.

### Install Pangolin

[Pangolin](https://github.com/stevenlovegrove/Pangolin) is a lightweight and portable library used for visualizing 3D data, camera views, and prototyping video-based programs. vS-Graphs has been tested with **Pangolin v0.8**.

```bash
# Clone
git clone --branch v0.9.1 --depth 1 https://github.com/stevenlovegrove/Pangolin.git

# Install using CMake
cd Pangolin
mkdir build && cd build
cmake ..
make -j
sudo make install
```

### Install Required ROS Libraries

Ensure that all necessary `ROS` dependencies are installed:

```bash
sudo apt-get update
sudo apt-get install -y ros-jazzy-rviz-visual-tools ros-jazzy-depth-image-proc ros-jazzy-backward-ros
sudo apt install ros-jazzy-pcl-ros
```

> 🛎️ Note: The current version of vS-Graphs supports **ROS2 Jazzy** and is primarily tested on Ubuntu 24.04.

### Install RealSense Library (Optional)

To use an Intel RealSense camera for **live mode** or **data collection**, you will need to install the necessary drivers and libraries. Please follow the detailed setup guide available in [RealSense Setup Instructions](/doc/RealSense/README.md) page. This includes steps for:

- Installing `librealsense`
- Verifying camera connection
- Enabling live streaming or data recording

### Install `aruco_ros` (Optional) <a id="aruco"></a>

vS-Graphs can operate independently of fiducial markers, but it also supports enhanced semantic labeling using detected markers (e.g., assigning room names) when a valid marker database is provided.

If you would like to use marker-driven data augmentation to the generated scene graph, install the [`aruco_ros`](https://github.com/pal-robotics/aruco_ros) library:

```bash
# Clone (ROS1 Noetic)
git clone -b humble-devel git@github.com:pal-robotics/aruco_ros.git
```

> 🛎️ Note: Use the sample modified version of the marker detector launch file available [here](doc/template_aruco_ros.launch). This setup ensures proper marker detection and integration with vS-Graphs.

### Install the In-house Scene Segmenter <a id="segmenter"></a>

vS-Graphs relies on a ROS-based **Panoptic Scene Segmentation** module, available at [scene_segment_ros](https://github.com/snt-arg/scene_segment_ros), to semantically identify **building components** (i.e., walls and ground surfaces). This module supports multiple segmentation backends, including:

- [**YOSO**](https://github.com/hujiecpp/YOSO) _(recommended for best performance)_
- [**PanopticFCN**](https://github.com/dvlab-research/PanopticFCN)

Clone the repository and set it ready for integration:

```bash
# Clone (ROS1 Noetic)
git clone -b ros2-jazzy git@github.com:snt-arg/scene_segment_ros.git

# Install Python packages
pip install -r src/requirements.txt

# Follow the rest installation guide in the repository
```

### 🦊 Integrating with Voxblox Skeleton (Optional) <a id="voxblox"></a>

#### Solution 1: Voxblox Skeleton for ROS2 Jazzy (Recommended)

To detect **structural elements** such as rooms and floors using the **free-space clustering** strategy, vS-Graphs integrates with `Voxblox Skeleton`, available in [this repository](https://github.com/snt-arg/voxblox_ros2_minimal).
This version is designed for **ROS2 Jazzy** and is fully compatible with the current vS-Graphs release.
A ready-to-use Docker environment is provided [in this path](voxblox/).

#### Solution 2: Voxblox Skeleton Relay for ROS1 Noetic

Alternatively, we provide a relay-based dockerized implementation of the original `Voxblox` in **ROS1 Noetic** (available in the [vS-Graphs Tools repository](https://github.com/snt-arg/vsgraphs_tools/tree/main)).
This solution contains a Dockerfile for building the complete `Voxblox` environment and a custom bridging utility that translates its input feed (`/camera/depth/points`) from vS-Graphs into **ROS1 Noetic** and output messages `/voxblox_skeletonizer/sparse_graph` into **ROS2 Jazzy**, compatible format for vS-Graphs.
For details and integration steps, see the [full guide](https://github.com/snt-arg/vsgraphs_tools/tree/main/Voxblox).

The procedure of using the tool is as below:

1. Download the tool [from here](https://github.com/snt-arg/vsgraphs_tools/blob/main/Voxblox/vox2ros.py) in your machine (if you are using vS-Graph's Docker, it will be there in `~/workspace/vsgraphs_tools/relay_jazzy.py` path)
2. Inside the relay's Docker environment, simply run the `mprocs` command:
```bash
docker exec -it voxblox_bridge bash

# Inside the tool's Docker environment
mprocs # Run Voxblox, and then the relay commands (keep the order)
```
3. Run the Python file as the voxblox (client) and depth feed (server) for vS-Graphs:
```bash
python /[path]/relay_jazzy.py --mode voxblox_client
python /[path]/relay_jazzy.py --mode pc_server

# In vS-Graphs Docker, simply choose them in `mprocs`
```

## ⚙️ II. Build the Project

After installing all the required dependencies and modules listed above, clone the repository and build it using `catkin`. To do so, initialize a new Catkin workspace, clone the repository inside it, and build it, as detailed below:

```bash
# Create and initialize a new Catkin workspace (ROS1 Noetic)
mkdir -p ~/vs_graphs_ws/src
cd ~/vs_graphs_ws/src

# Clone the vS-Graphs repository into the src folder
git clone git@github.com:snt-arg/visual_sgraphs.git

# Build the workspace
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
rosdep update
```
