#!/bin/bash

# if [ -z "${ROS__BRIDGE:-}" ]; then
#     echo "ROS__BRIDGE is not set!"
#     echo "Please source your ports.env / set_ports.sh first."
#     exit 1
# else
#     echo "ROS__BRIDGE=$ROS__BRIDGE"
# fi

CONTAINER_NAME="yolo_container"
DOCKER_IMAGE="ispsae/amr_yolo:latest"
ROS_WS="/ros2_ws"
STATUS=$(docker inspect -f '{{.State.Status}}' "$CONTAINER_NAME" 2>/dev/null || echo "not found")

if [ "$STATUS" = "running" ]; then
    docker exec -it $CONTAINER_NAME bash
elif [ "$STATUS" = "exited" ]; then
    docker start $CONTAINER_NAME
    docker exec -it $CONTAINER_NAME bash
else
    docker run \
        -it \
        --name $CONTAINER_NAME \
        --network host \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        -e ROS_IP=$(hostname -I | awk '{print $1}') \
        -e ROS_WS=$ROS_WS \
        -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
        -e ROS__BRIDGE=$ROS__BRIDGE \
        -v ./rospkg:$ROS_WS/src \
        -w $ROS_WS \
        $DOCKER_IMAGE
fi
