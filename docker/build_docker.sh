#!/bin/bash
cp Dockerfile ..
cp run_blueye.sh ..
cd ..
DOCKER_BUILDKIT=1 docker build -t ghcr.io/labust/blueye-manual-control:latest $* .
rm Dockerfile
rm run_blueye.sh
cd docker
