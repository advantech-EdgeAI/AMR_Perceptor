#!/bin/bash

if [ -z "${AI__LLAMACPP:-}" ]; then
    echo "AI__LLAMACPP is not set!"
    echo "Please source your ports.env / set_ports.sh first."
    exit 1
else
    echo "AI__LLAMACPP=$AI__LLAMACPP"
fi

CONTAINER_NAME="open_webui_container"
DOCKER_IMAGE="ghcr.io/open-webui/open-webui:main-slim"
STATUS=$(docker inspect -f '{{.State.Status}}' "$CONTAINER_NAME" 2>/dev/null || echo "not found")

if [ "$STATUS" = "running" ]; then
    docker exec -it $CONTAINER_NAME bash
elif [ "$STATUS" = "exited" ]; then
    docker start $CONTAINER_NAME
    docker exec -it $CONTAINER_NAME bash
else
    docker run \
        -d \
        --rm \
        --network host \
        --gpus all \
        -e WEBUI_AUTH=False \
        -v open-webui:/app/backend/data \
        --name $CONTAINER_NAME \
        $DOCKER_IMAGE
fi
