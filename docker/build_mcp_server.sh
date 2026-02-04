#!/bin/bash

docker build \
    -t ispsae/amr_mcp:latest \
    -f ./Dockerfile.mcp \
    .
