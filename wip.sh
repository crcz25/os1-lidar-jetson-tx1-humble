docker build --no-cache \
-f docker/Dockerfile \
-t ouster_lidar_image:latest \
--target full \
--build-arg AMENT_WORKSPACE_DIR=/ament_ws --build-arg USERNAME=devuser --build-arg USER_UID=1000 --build-arg USER_GID=1000 .

docker save -o ouster_lidar_image.tar ouster_lidar_image:latest

docker load -i ouster_lidar_image.tar

docker-compose up -d

docker-compose --progress plain -f ./docker/docker-compose.yml up
