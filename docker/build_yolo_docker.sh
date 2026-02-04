#!/bin/bash

docker build \
    -t ispsae/amr_yolo:latest \
    -f ./Dockerfile.yolo \
    .
