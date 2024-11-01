# OS1 LiDAR Setup on Jetson TX1/Nano with ROS 2 Humble in Docker
This repository provides a Docker-based setup for using the OS1 LiDAR with the Jetson TX1/Nano platform running ROS 2 Humble.

# Prerequisites
1. Install Docker Compose
   - Follow the official [Docker Compose installation instructions](https://docs.docker.com/compose/install/standalone/), making sure to use the aarch64 version for ARM-based Jetson devices.
2. Set up NVIDIA Runtime for Docker
   - Verify NVIDIA runtime is enabled:
        ```bash
        docker info | grep -i runtime
        ```
    - If NVIDIA runtime is not set up, edit the Docker daemon configuration:
        ```bash
        sudo nano /etc/docker/daemon.json
        ```
    - Add or modify the file as follows:
        ```bash
        {
        "runtimes": {
            "nvidia": {
                "path": "nvidia-container-runtime",
                "runtimeArgs": []
            }
        },
        "default-runtime": "nvidia"
        }
        ```
    - Restart Docker to apply changes:
        ```bash
        sudo systemctl restart docker
        docker info | grep -i runtime
        ```

# Building and Running the Container

# 1. Build the Docker Image
- To build the Docker image, use:
```bash
docker-compose --progress plain -f ./docker/docker-compose.yml build
```

# 2. Start the Container
- To start the container, run:
```bash
docker-compose --progress plain -f ./docker/docker-compose.yml up
```

# Installing Custom ROS Packages
This image is based on the [Jetson ROS containers](https://github.com/dusty-nv/jetson-containers/tree/master/packages/ros), which install ROS from source. To maintain compatibility, install additional ROS packages by building from source rather than using apt. Use the helper script provided:
- Script: `docker/ros2_install.sh` or `/usr/local/bin/ros2_install.sh` (within the container).
- Usage: Supply either a list of ROS package names or a Git repository URL to build and install in a ROS workspace.

Examples:
```bash
# adds foxglove to ROS_ROOT (under /opt/ros)
/ros2_install.sh foxglove_bridge

# adds jetson-inference nodes under /ros2_workspace
ROS_WORKSPACE=/ros2_workspace /ros2_install.sh https://github.com/dusty-nv/ros_deep_learning
```

# Configuring the Ouster LiDAR IP and Hostname
1. Set up the `.env` file located in `docker/.env`.
   - Define the IP and hostname for the Ouster LiDAR. The hostname should follow a convention with the device's serial number.
        ```bash
        OS_HN="os-XXXXXXXXXXXX.local"
        OS_IP="XXX.XXX.XXX.XXX"
        ```
2. Verify the Setup within the container:
    ```bash
        # Ping using hostname
        devuser@devuser-jetson:/ament_ws$ ping -4 -c3 os-122144001514.local
        PING os-122144001514.local (169.254.207.60) 56(84) bytes of data.
        64 bytes from os-122144001514.local (169.254.207.60): icmp_seq=1 ttl=64 time=0.622 ms
        64 bytes from os-122144001514.local (169.254.207.60): icmp_seq=2 ttl=64 time=0.182 ms
        64 bytes from os-122144001514.local (169.254.207.60): icmp_seq=3 ttl=64 time=0.239 ms

        --- os-122144001514.local ping statistics ---
        3 packets transmitted, 3 received, 0% packet loss, time 2052ms
        rtt min/avg/max/mdev = 0.182/0.347/0.622/0.195 ms

        # Ping using IP
        PING 169.254.207.60 (169.254.207.60) 56(84) bytes of data.
        64 bytes from 169.254.207.60: icmp_seq=1 ttl=64 time=0.545 ms
        64 bytes from 169.254.207.60: icmp_seq=2 ttl=64 time=0.330 ms
        64 bytes from 169.254.207.60: icmp_seq=3 ttl=64 time=0.190 ms

        --- 169.254.207.60 ping statistics ---
        3 packets transmitted, 3 received, 0% packet loss, time 2042ms
        rtt min/avg/max/mdev = 0.190/0.355/0.545/0.146 ms
    ```

# References
For more information, refer to the original Jetson ROS containers documentation: https://github.com/dusty-nv/jetson-containers/tree/master/packages/ros
