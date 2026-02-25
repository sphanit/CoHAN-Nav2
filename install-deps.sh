#!/bin/bash 
sudo rosdep init
rosdep update
git submodule update --init --recursive
rosdep install --from-paths src --ignore-src -r -y
