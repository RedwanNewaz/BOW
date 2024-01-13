#!/usr/bin/env bash 
# To forward x11 in macos follow this tutorial https://gist.github.com/cschiewek/246a244ba23da8b9f0e7b11a68bf3285
docker run --rm -it -v $(pwd):/home/BOW \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=${HOSTNAME}:0 \
    -w /home/BOW osrf/ros:humble-desktop bash