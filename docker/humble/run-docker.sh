#! /usr/bin/env bash

set -o errexit
set -o nounset

source env.sh

xhost +local:docker > /dev/null

args=(
    --rm
    --interactive
    --tty
    --privileged 
    --network=host
    --env TERM=xterm-256color
    --hostname="$HOSTNAME"
    --workdir="$PWD"

    --ipc=host
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:ro"
    --volume="/dev/dri:/dev/dri:ro"
    --env DISPLAY="$DISPLAY"

    --volume="$HOME:$HOME"

    --runtime=nvidia
    --gpus all
    --env NVIDIA_DRIVER_CAPABILITIES="all"

    "$IMAGE"
    "$SHELL"
    )

if ! docker container inspect -f '{{.State.Running}}' "$CONTAINER_NAME" &> /dev/null; then
    docker run "${args[@]}"		
fi

