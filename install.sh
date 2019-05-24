#!/usr/bin/env bash
sudo apt-get -y install libgsl-dev
sudo apt-get install -y cmake libeigen3-dev libopenblas-dev  liblapack-dev libarpack* libarmadillo*
cd /catkin_ws/src/lms1xx_perception/libeigen-3.3.7 && mkdir build && cd build && cmake .. && make install
