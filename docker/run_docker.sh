#!/bin/bash
docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    ghcr.io/labust/blueye-manual-control /bin/bash

export containerId=$(docker ps -l -q)

xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $containerId`
docker start $containerId
