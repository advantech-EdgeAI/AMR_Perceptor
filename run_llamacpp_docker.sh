#!/bin/bash

if [ -z "${AI__LLAMACPP:-}" ]; then
    echo "AI__LLAMACPP is not set!"
    echo "Please source your ports.env / check_and_set_ports.sh first."
    exit 1
else
    echo "AI__LLAMACPP=$AI__LLAMACPP"
fi

CONTAINER_NAME="llamacpp_server_container"
DOCKER_IMAGE="ispsae/amr_llamacpp"
STATUS=$(docker inspect -f '{{.State.Status}}' "$CONTAINER_NAME" 2>/dev/null || echo "not found")
LLAMACPP_CMD="./llama.cpp/build/bin/llama-server \
                --model gpt-oss-20b-Q4_K_M.gguf \
                --port $AI__LLAMACPP \
                --ctx-size 16384 \
                --gpu-layers 100 \
                --flash-attn auto \
                --jinja \
                -b 1024 \
                -ub 1024 \
                --temp 0.1 \
                --top-p 0.9 \
                --repeat-penalty 1.1 \
                --n-predict 256 \
                --mirostat 0"

if [ "$STATUS" = "running" ]; then
    docker stop $CONTAINER_NAME
elif [ "$STATUS" = "exited" ]; then
    docker rm $CONTAINER_NAME
fi

docker run \
    -d \
    --rm \
    --gpus all \
    --name $CONTAINER_NAME \
    --runtime nvidia \
    --network host \
    --health-cmd="curl -f http://localhost:$AI__LLAMACPP/health || exit 1" \
    --health-interval=10s \
    --health-retries=30 \
    $DOCKER_IMAGE \
    $LLAMACPP_CMD
