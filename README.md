# Tramcar LMS1xx Perception 

[![Build Status][travis-img]][travis-link]
[![Codacy Badge][codacy-img]][codacy-link]

This is part of our tramcar perception system. It generate a fused grid map for obstacles with four [SICK LMS151](https://www.sick.com/ag/en/detection-and-ranging-solutions/2d-lidar-sensors/lms1xx/lms151-10100/p/p141840). 

Check out a video that shows all objects with the distance of less than 15 meters: 


[![Obstacle detection of tramcar](https://s2.ax1x.com/2019/05/24/ViDdJg.png)](https://v.qq.com/x/page/j0874k4u7tb.html "Obstacle detection of tramcar")

## Prerequisites 

### Set up workspace and catkin 
Regardless of your system you will need to do the following steps:
```bash
cd <catkin_ws>            # navigate to the workspace
mkdir src                 # create src dir if you don't have it already
cd src
# Now you just need to clone the repo:
git clone https://github.com/zhearing/lms1xx_perception.git
```
### System requirements

**Warning**: The driver of [LMS1xx](https://github.com/clearpathrobotics/LMS1xx) is **Not Available** under Ubuntu 18.04, you need to fix the bug by yourself.

You will need [ROS](http://www.ros.org/install/), [CMake](www.cmake.org/), [Eigen3](eigen.tuxfamily.org/), [Armadillo](http://arma.sourceforge.net/), [RViz plugins](http://wiki.ros.org/rviz). The following sections contain an installation command for various Ubuntu systems (click folds to expand):

<details>
<summary>Ubuntu 14.04</summary>

#### Install these packages:

```bash
sudo apt install -y ros-indigo-desktop-full libeigen3-dev libopenblas-dev liblapack-dev libarpack* libarmadillo*
```
</details>

<details>
<summary>Ubuntu 18.04</summary>

#### Install these packages:

```bash
sudo apt install -y ros-melodic-desktop-full ros-melodic-rviz*
```
</details>

You can easily install these packages using `sudo bash install.sh`. Script is include in [install.sh](https://github.com/zhearing/lms1xx_perception/blob/master/install.sh).


## How to build?

This is a catkin package. So we assume that the code is in a catkin workspace and CMake knows about the existence of Catkin. It should be already taken care of if you followed the instructions [here](#set-up-workspace-and-catkin). Then
you can build it from the project folder:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release -j8
```

## How to run?

### Run online
```bash
roslaunch lms1xx_perception four_scanners.launch
roslaunch lms1xx_perception.launch
```
### Run offline
Note: Don't forget to use simulation time when republishing from a bag file: [Using Sim Time](http://wiki.ros.org/Clock#Using_Simulation_Time_from_the_.2BAC8-clock_Topic).
```bash
roslaunch lms1xx_perception.launch
```

## Authors 

* **Zeyu Zhong** - *Initial work* - The single SICK LMS1xx ROS driver is originally from [*@clearpathrobotics*](https://github.com/clearpathrobotics/LMS1xx). The obstacle_detector package is originally from [*@tysik*](https://github.com/tysik/obstacle_detector).

## License

This project is licensed under the LGPL-3.0 License.

[travis-img]: https://travis-ci.com/zhearing/lms1xx_perception.svg?token=2AAv6RKxhQmpctyh8xz6&branch=master
[travis-link]: https://travis-ci.com/zhearing/lms1xx_perception
[codacy-img]: https://api.codacy.com/project/badge/Grade/e9dd9e8a49474b348bb9ea3fe592d5ef
[codacy-link]: https://app.codacy.com/app/zhearing/lms1xx_perception/dashboard
