#!/bin/bash

type=${1:-Release}

# Build the selected packages (for development)
if command -v colcon &> /dev/null
then
    # colcon build --symlink-install --packages-skip cohan_tutorial --cmake-args -DCMAKE_BUILD_TYPE=$type -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -Wno-dev
    colcon build --symlink-install --packages-skip cohan_tutorial --cmake-args -DCMAKE_BUILD_TYPE=$type -Wno-dev
else
    echo "colcon not found!"
    exit 1
fi
