# 🚀 Run vS-Graphs using Docker

## ✅ I. Set Up NVIDIA Container Toolkit (Ubuntu)

You might need to install `Nvidia`'s container toolkit to

```bash
# Detect your Ubuntu distribution
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)

# Add the NVIDIA GPG key
curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

# Add the NVIDIA container repository
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install the toolkit
sudo apt update
sudo apt install -y nvidia-container-toolkit

# Configure the Docker runtime (inside docker folder)
sudo nvidia-ctk runtime configure --runtime=docker

# Restart Docker
sudo systemctl restart docker
```

## ⚙️ II. Build

You can build the Docker image using the command `docker compose build`. Otherwise, and if you want to manually build the Docker image, run the following command within this directory:

```bash
docker build --ssh default -t vsgraphs -f Noetic.Dockerfile .
```

> 🛎️ Tip: Please note that your Github authentication keys might be named differently depending on the encryption algorithm. If above does not work, try replacing `id_rsa` with `id_ed25519`, in the above command.

## 🚀 III. Run

### III-A. Run the Docker Image

Use one of the below options:

```bash
# [Option I] using Docker Compose
docker compose up -d

# [Option II] manual Docker image run
docker run -it -d --privileged --name vsgraphs_ros2 -e DISPLAY=$DISPLAY -e XAUTHORITY=$XAUTHORITY -v /tmp/.X11-unix:/tmp/.X11-unix -v $XAUTHORITY:$XAUTHORITY vsgraphs_ros2

# [Option III] using devcontainers
# in `VSCode`, select "Reopen in Container"
```

> 🛎️ Tip: If you use **Docker Compose**, make sure to properly configure `Working Directories` (maps your project folder from the host to the container to ensure code and configuration changes persist) and `Data Directories` (mount the folder containing your datasets (e.g., ROS bags) so they are accessible and runnable inside the container).

### III-B. Run the Container

```bash
docker exec -it vsgraphs_ros2 bash

# Inside the container
ros2 launch vs_graphs rgbd.launch.py
```

You can even use the **mprocs** tool provided inside the Docker image, by simply running `mprocs` (an alias for `mprocs -c [path]/visual_sgraphs/config/mprocs.yml`) and choose the prepared command sets there.