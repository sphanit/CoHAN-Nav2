#!/usr/bin/env bash

set -o errexit
set -o nounset
set -ex

source ./env.sh

docker buildx build --rm=true --progress=plain --build-arg="HOME=$HOME" --build-arg="UID=$HOST_UID" --build-arg="GID=$HOST_GID" --build-arg="USER_NAME=$CONTAINER_NAME" -t "$IMAGE" .
docker system prune -f
