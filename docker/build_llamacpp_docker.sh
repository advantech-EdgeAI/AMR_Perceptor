#!/bin/bash

docker build \
    -t ispsae/amr_llamacpp:latest \
    -f ./Dockerfile.llamacpp \
    .
