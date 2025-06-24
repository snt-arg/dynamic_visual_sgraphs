FROM nvidia/cuda:12.9.0-cudnn-devel-ubuntu24.04
ARG DEBIAN_FRONTEND=noninteractive

# User and group setup
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Deletes user if already in container
RUN if id -u $USER_UID ; then userdel "$(id -un $USER_UID)" ; fi

##### Environment variables #####
ENV CUDA_HOME=/usr/local/cuda
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=jazzy

##### Essential packages #####
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python-is-python3 \
    git \
    openssh-client \
    wget \
    vim \
    curl \
    libeigen3-dev \
    build-essential \
    locales \
    software-properties-common \
    lsb-release \
    gnupg2 && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

##### ROS 2 Jazzy setup #####

# Add universe repository
RUN add-apt-repository universe

# Install ros-apt-source package
RUN ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    apt install -y /tmp/ros2-apt-source.deb

# Install dev tools and ROS
RUN apt update && apt upgrade -y && \
    apt install -y ros-dev-tools ros-${ROS_DISTRO}-desktop

# Source ROS setup globally
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

# rosdep initialization (should run as root before switching user)
RUN rosdep init && rosdep update

##### Clean up to reduce image size
RUN rm -rf /var/lib/apt/lists/* /tmp/*

# Create new user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

##### Python environment setup #####
# PyTorch and related libraries - networkx needs to be installed first because of version issue
# RUN pip3 install networkx==3.1
# RUN pip3 install torch==2.0.1+cu118 -f  https://download.pytorch.org/whl/torch_stable.html
# RUN pip3 install torchvision==0.15.2+cu118 --extra-index-url https://download.pytorch.org/whl/cu118

RUN pip3 install --break-system-packages networkx==3.1
RUN pip3 install --break-system-packages torch==2.3.1+cu121 -f https://download.pytorch.org/whl/torch_stable.html
# RUN pip3 install --break-system-packages torchvision==0.16.1+cu121 --extra-index-url https://download.pytorch.org/whl/cu121
RUN pip3 install --break-system-packages torchvision==0.18.1+cu121 --extra-index-url https://download.pytorch.org/whl/cu121


# detectron and CLIP
# Compute Capability 7.5 for T600 (SnT laptop) and 7.0 for V100 (HPC - Iris)
ARG TORCH_CUDA_ARCH_LIST="7.5;7.0+PTX"
ENV FORCE_CUDA="1"
RUN pip3 install --break-system-packages 'git+https://github.com/facebookresearch/detectron2.git'
RUN pip3 install --break-system-packages 'git+https://github.com/openai/CLIP.git'

##### SSH keys for GitHub #####

# Define the SSH keys as build arguments for latter mounting
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

##### Clone repositories #####
# Pangolin
# RUN apt-get install libepoxy-dev -y
# RUN apt-get update && apt-get install -y libepoxy-dev
# WORKDIR /opt/
# # RUN git clone --branch v0.8 --depth 1 https://github.com/stevenlovegrove/Pangolin.git && \
# RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
#     cd Pangolin && \
#     mkdir build && cd build && \
#     cmake .. && \
#     make -j && \
#     make install

# Pangolin
# RUN apt-get update && apt-get install libepoxy-dev -y
# WORKDIR /opt/
# RUN git clone --branch v0.8 --depth 1 https://github.com/stevenlovegrove/Pangolin.git && \
#     cd Pangolin && \
#     mkdir build && cd build && \
#     cmake .. && \
#     make -j && \
#     make install

# Pangolin - Use a more recent version that works with modern compilers
RUN apt-get update && apt-get install -y \
    libepoxy-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    freeglut3-dev \
    libglew-dev \
    cmake \
    build-essential \
    git

WORKDIR /opt/
RUN git clone --branch v0.9.1 --depth 1 https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    mkdir build && cd build && \
    cmake .. && \
    make -j && \
    make install

# Cmake
ARG version=3.22
ARG build=1
WORKDIR /tmp
RUN wget https://cmake.org/files/v$version/cmake-$version.$build.tar.gz

RUN tar -xzvf cmake-$version.$build.tar.gz
WORKDIR /tmp/cmake-$version.$build
RUN ./bootstrap
RUN make -j8
RUN make install

# # ROS packages: visual sgraphs, semantic segmenter, aruco ros
RUN mkdir -p /workspace/src
WORKDIR /workspace/src/

# # Mount the SSH keys for cloning private repositories
# RUN --mount=type=ssh git clone git@github.com:snt-arg/visual_sgraphs.git
# RUN --mount=type=ssh git clone git@github.com:snt-arg/scene_segment_ros.git
# RUN --mount=type=ssh git clone git@github.com:snt-arg/scene_segment_ros2.git
# RUN --mount=type=ssh git clone -b noetic-devel git@github.com:pal-robotics/aruco_ros.git
# RUN --mount=type=ssh git clone -b humble-devel git@github.com:pal-robotics/aruco_ros.git

# # other libraries
WORKDIR /workspace/src/visual_sgraphs/docker
# RUN pip3 install -r requirements.txt
# WORKDIR /workspace/src/

RUN pip3 install --break-system-packages --ignore-installed typing_extensions \
    networkx==3.1 \
    ultralytics==8.0.120 \
    matplotlib>=3.2.2 \
    opencv-python==4.6.0.66 \
    Pillow>=7.1.2 \
    PyYAML>=5.3.1 \
    requests>=2.23.0 \
    scipy>=1.4.1 \
    tqdm>=4.64.0 \
    pandas>=1.1.4 \
    seaborn>=0.11.0 \
    gradio==4.11.0 \
    wandb \
    transformers \
    ftfy \
    regex \
    timm \
    evo
WORKDIR /workspace/src/

# # for ROS package: mav_voxblox_planning
# RUN --mount=type=ssh git clone git@github.com:snt-arg/mav_voxblox_planning.git
# RUN --mount=type=ssh wstool init . ./mav_voxblox_planning/install/install_ssh.rosinstall
# RUN --mount=type=ssh wstool update

# # download the yoso checkpoint
# RUN wget https://github.com/hujiecpp/YOSO/releases/download/v0.1/yoso_res50_coco.pth
# RUN mv yoso_res50_coco.pth /workspace/src/scene_segment_ros/include/

# # build the workspace
WORKDIR /workspace/
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build -j12 -DCMAKE_BUILD_TYPE=Release && rosclean purge -y"
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && rosdep update"
# RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --no-warn-unused-cli && rosdep update"


##### Miscalleanous #####
RUN ldconfig
# RUN echo 'export PS1="[\u@\h \W] 🐳 "' >> /home/asier/.bashrc
RUN echo 'export PS1="[\u@\h \W] 🐳 "' >> /home/$USERNAME/.bashrc

#### For cmakelist of vsual_sgraphs #####
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y ros-jazzy-rviz-visual-tools
# RUN sudo apt update && sudo apt install ros-jazzy-rviz-visual-tools
RUN sudo apt install ros-jazzy-pcl-ros

##### Clean up #####
# remove the apt list files
RUN rm -rf /var/lib/apt/lists/*

# remove packages no longer needed
RUN apt-get clean && apt-get autoremove -y

# remove the ssh keys
RUN rm -rf /root/.ssh/

##### Build entrypoint #####
RUN echo "#!/bin/bash" >> /entrypoint.sh \
    && echo "echo \"source /opt/ros/$ROS_DISTRO/setup.bash\" >> ~/.bashrc" >> /entrypoint.sh \
    && echo "echo \"source /home/$USERNAME/ros2_ws/install/setup.bash\" >> ~/.bashrc" >> /entrypoint.sh \
    && echo 'exec "$@"' >> /entrypoint.sh \
    && chmod a+x /entrypoint.sh

WORKDIR /workspace/

ENTRYPOINT ["/entrypoint.sh"]

USER $USERNAME
CMD ["/bin/bash"]
SHELL ["/bin/bash"]

