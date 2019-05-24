#!/usr/bin/env bash
sudo apt-get -y install libgsl-dev
sudo apt-get -y install cmake libopenblas-dev liblapack-dev libarpack* libarmadillo*
cd ~/catkin_ws/src/lms1xx_perception/lib/eigen-3.3.7 && mkdir build && cd build && cmake .. && make install && cd /usr/local/include && sudo ln -sf eigen3/Eigen Eigen
