#!/bin/bash

# if [ -z "${ROS__BRIDGE:-}" ] || [ -z "${ROS__MCP:-}" ]; then
#     echo "ROS__BRIDGE or ROS__MCP is not set!"
#     echo "Please source your ports.env / set_ports.sh first."
#     exit 1
# else
#     echo "ROS__BRIDGE=$ROS__BRIDGE"
#     echo "ROS__MCP=$ROS__MCP"
# fi

CONTAINER_NAME="mcp_container"
DOCKER_IMAGE="ispsae/amr_mcp:latest"
STATUS=$(docker inspect -f '{{.State.Status}}' "$CONTAINER_NAME" 2>/dev/null || echo "not found")

if [ "$STATUS" = "running" ]; then
    docker exec -it $CONTAINER_NAME bash
elif [ "$STATUS" = "exited" ]; then
    docker start $CONTAINER_NAME
    docker exec -it $CONTAINER_NAME bash
else
    docker run \
        -d \
        -it \
        --name $CONTAINER_NAME \
        --network host \
        --health-cmd "nc -z localhost $ROS__MCP || exit 1" \
        --health-interval=10s \
        --health-timeout=3s \
        --health-retries=30 \
        -e ROS__BRIDGE=$ROS__BRIDGE \
        $DOCKER_IMAGE \
        mcpo --port $ROS__MCP -- uv run server.py
fi
