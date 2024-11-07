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

# Building and Running the Containers

## Configure the .env File
- Set the following parameters in `docker/.env`:
  - AMENT_WORKSPACE_DIR: ROS 2 workspace directory. Default is `/ament_ws`.
  - ROS_DOMAIN_ID: ROS 2 network domain ID. Default is `10`.
  - UID: Host machine user ID. Default is `1000`.
  - GID: Host machine group ID. Default is `1000`.
  - JETSON_IP: Jetson IP address. Default is `XXX.XXX.XXX.XXX`.
  - DESKTOP_IP: Desktop machine IP address. Default is `XXX.XXX.XXX.XXX`.
  - OS_HN: OS1 LiDAR hostname. Default is `os-XXXXXXXXXXXX.local`.
  - OS_IP: OS1 LiDAR IP address. Default is `XXX.XXX.XXX.XXX`.

## Jetson Nano
### 1. Build the Docker Image
- Build the Docker image for the Jetson Nano with:
  - On an x86_64 machine, run:
      ```bash
      docker compose --progress plain -f docker/docker-compose-prebuild.yml build
      ```
    - **Note**: If you’re building the image on an x86_64 machine, install `qemu-user-static` to enable multi-architecture builds:
        ```bash
        sudo apt-get install qemu-user-static
        ```
  - On a Jetson Nano, run (not tested):
      ```bash
      docker-compose --progress plain -f docker/docker-compose-prebuild.yml build
    ```
### 2. Move and load the image to the Jetson Nano:
  - Save the image to a tar file:
      ```bash
      docker save -o ros2_humble_jetson.tar ros2_humble_jetson:latest
      ```
  - Transfer the tar file to the Jetson Nano:
      ```bash
      rsync -a -P \
        ros2_humble_jetson.tar \
        <jetson_user>@<jetson_ip>:<destination_path_on_jetson>
      ```
  - Load the image in the Jetson Nano:
      ```bash
      docker load -i <destination_path_on_jetson>/ros2_humble_jetson.tar
      ```

### 3. Start the Container
- To start the container that runs in the Jetson, run:
    ```bash
    docker-compose --progress plain -f ./docker/docker-compose-jetson.yml up
    ```

## Desktop
### 1. Build the Docker Image
- Build the Docker image for the Desktop (used to run RViz and other tools unavailable on the Jetson) with:
    ```bash
    docker compose --progress plain -f docker/docker-compose-desktop.yml build
    ```

### 2. Start the Container
- To start the container that runs in the Desktop, run:
    ```bash
    docker compose --progress plain -f docker/docker-compose-desktop.yml up
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

# Testing the installation
## Connect to a container
- To access the bash inside any container use the following:
    ```bash
    docker exec -it <container_name> bash
    ```

## Listener/Talker demo
To verify the installation, it’s recommended to test communication using the talker and listener nodes from the demo_nodes_cpp package.
- Inside any of the containers, make sure to source the ROS 2 environment:
    ```bash
    source /opt/ros/humble/setup.bash
    ```
- In one container, start the `talker` node:
    ```bash
    ros2 run demo_nodes_cpp talker
    ```
- In the other container, start the `listener` node:
    ```bash
    ros2 run demo_nodes_cpp listener
    ```
- You should see the `listener` node printing messages published by the `talker` node. In any direction, either from the Jetson to the Desktop or vice versa, the nodes should communicate successfully. For debugging purposes, CycloneDDS is configured to print debug messages to the console by default. To disable this, simply comment out the `<Tracing>` tag in the CycloneDDS configuration file located at `dds/cyclone.xml`.

## OS1 LiDAR Test
- In the Jetson container, start the Ouster ROS driver by running the following command:
    ```bash
    ros2 launch ouster_ros driver.launch.py \
    params_file:=src/ouster_config.yaml \
    viz:=false
    ```
This command launches the Ouster driver node, configured with the parameters specified in `src/ouster_config.yaml`. The `viz:=false` option disables built-in visualization.
- On the desktop container, you can either:
  - Launch RViz to visualize LiDAR data, or list available ROS 2 topics to confirm the LiDAR driver is publishing data.
  - To list available topics, run:
    ```bash
    ros2 topic list
    ```
    Example output:
    ```bash
    /ouster/imu
    /ouster/imu_packets
    /ouster/lidar_packets
    /ouster/metadata
    /ouster/nearir_image
    /ouster/os_driver/transition_event
    /ouster/points
    /ouster/range_image
    /ouster/reflec_image
    /ouster/scan
    /ouster/signal_image
    /parameter_events
    /rosout
    /tf_static
    ```

# References
For more information, refer to the original Jetson ROS containers documentation: https://github.com/dusty-nv/jetson-containers/tree/master/packages/ros
