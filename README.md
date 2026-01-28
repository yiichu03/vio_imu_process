README
======

Welcome to SWIFT VIO, short for Sliding WIndow FilTers for Visual Inertial Odometry.
There are several sliding window filters and smoothers for visual inertial odometry.

* SlidingWindowFilter implements a keyframe-based sliding window filter, which can be viewed as a hybrid of MSCKF and EKF-SLAM enhanced by the keyframe scheme.
* FixedLagSmoother implements a fixed-lag smoother based on gtsam. Being fixed-lag, it does not use keyframes.
* RiFixedLagSmoother is a fixed-lag smoother that uses the right invariant errors.
* Estimator is basically the original sliding window smoother in OKVIS. Slight changes have been made to decouple the estimation backend and feature association frontend.

All estimators are derived from the EstimatorBase which keeps the necessary data structures largely borrowed from OKVIS.

Table of Contents
- [README](#readme)
  - [Installation Guide](#installation-guide)
    - [Dependencies](#dependencies)
  - [Install on mamba ros noetic venv ros\_env](#install-on-mamba-ros-noetic-venv-ros_env)
    - [Build swift\_vio](#build-swift_vio)
    - [Build Tests for a package](#build-tests-for-a-package)
    - [Run Tests](#run-tests)
    - [Error while loading shared libraries: libmetis-gtsam.so](#error-while-loading-shared-libraries-libmetis-gtsamso)
  - [Simulation test](#simulation-test)
  - [Process Datasets](#process-datasets)
    - [Process Data from ROS Topics](#process-data-from-ros-topics)
    - [Process a ROS Bag Synchronously](#process-a-ros-bag-synchronously)
    - [Process Smartphone Data](#process-smartphone-data)
  - [Input and Output Description](#input-and-output-description)
    - [Parameter Description](#parameter-description)
    - [Configuration Files](#configuration-files)
    - [Coordinate Frames](#coordinate-frames)
  - [Development](#development)


## Installation Guide

### Dependencies

This catkin package requires the following dependencies,

* ROS (currently supported: kinetic, melodic, noetic). 
Read the ROS installation [instructions](http://wiki.ros.org/melodic/Installation/Ubuntu).

* google-glog + gflags,

```
sudo apt-get install libgoogle-glog-dev
```

* The following should get installed through ROS anyway:

```
sudo apt-get install libatlas-base-dev libeigen3-dev libsuitesparse-dev 
sudo apt-get install libboost-dev libboost-filesystem-dev
```

* catkin tools
```
sudo apt-get install python-catkin-tools
```

* gtest (**No action is required**)

The ros melodic desktop distro will install the source files for the three packages by default, googletest libgtest-dev, and google-mock.
The googletest package includes source for both googletest and googlemock.
*You do not need to cmake and install gtest libraries to /usr/lib.*.

* Eigen (**For Ubuntu 16**)

The system wide Eigen library in Ubuntu >= 18 is OK for swift_vio.
However, in Ubuntu 16, the system wide Eigen library (usually of version 3.2) does not 
meet the requirements of ceres solver used by swift_vio.
Therefore, a newer Eigen library (newer than 3.3.4) should be downloaded from 
[here](https://github.com/eigenteam/eigen-git-mirror/releases)
and installed in a local directory say $HOME/Documents/slam_devel by the below commands.

```
mkdir -p $HOME/slam_src
cd $HOME/slam_src
wget https://github.com/eigenteam/eigen-git-mirror/archive/3.3.4.zip
unzip 3.3.4.zip
mv eigen-git-mirror-3.3.4 eigen-3.3.4
cd $HOME/slam_src/eigen-3.3.4
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX="$HOME/Documents/slam_devel"
make install
```

* sophus

```
git clone https://github.com/stevenlovegrove/Sophus.git
cd Sophus

mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/Documents/slam_devel

make -j $(nproc)
make install
```


* gtsam (Optional)

```
sudo apt-get install libtbb-dev
cd $HOME/Documents/slam_src
git clone https://github.com/borglab/gtsam.git --recursive
cd gtsam

git checkout 8c98eefb24f846267119f7f81466dd660f195b06
# 6c85850147751d45cf9c595f1a7e623d239305fc
# 342f30d148fae84c92ff71705c9e50e0a3683bda(previously tested commit)
mkdir build && cd build

# GTSAM can be installed locally, e.g., at $HOME/Documents/slam_devel, but 
# /usr/local is recommended as it has no issue when debugging swift_vio in QtCreator.

cmake -DCMAKE_INSTALL_PREFIX=$HOME/Documents/gtsam_devel -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_BUILD_UNSTABLE=ON -DGTSAM_USE_SYSTEM_EIGEN=ON ..
# -DGTSAM_TANGENT_PREINTEGRATION=OFF -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON
# -DEIGEN3_INCLUDE_DIR=$HOME/Documents/slam_devel/include/eigen3 -DEIGEN_INCLUDE_DIR=$HOME/Documents/slam_devel/include/eigen3 # for Ubuntu 16
# In Ubuntu 16, to circumvent the incompatible system-wide Eigen, passing the local Eigen by EIGEN_INCLUDE_DIR is needed.
# -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF # for linux, https://github.com/gisbi-kim/FAST_LIO_SLAM/issues/8


make -j $(nproc) check # (optional, runs unit tests)
sudo make -j $(nproc) install
```

## Install on mamba ros noetic venv ros_env
First, create the robostack ros noetic venv ros_env following offical instructions.

Second, build ceres solver using the ros_env glog, and set CMAKE_INSTALL_PREFIX to current_ws/devel

```
cd ceres-solver
git checkout ce9e902b86be52d347a142429be1dbe3f326de0b
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/home/jhuai/Documents/slam_devel/ -DCMAKE_CUDA_ARCHITECTURES=89 \
    -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc
```

Third, build swift_vio with catkin build, using ros_env glog, and the locally installed ceres solver.

```
catkin build sliding_window_estimator
```

### Build swift_vio

```
cd swift_vio_ws/

export ROS_DISTRO=melodic # alternatively, kinetic
catkin init
catkin config --merge-devel # Necessary for catkin_tools >= 0.4.
catkin config --extend /opt/ros/$ROS_DISTRO
catkin config --cmake-args -DBUILD_TESTS=ON \
 -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM
# -DEIGEN3_INCLUDE_DIR=$HOME/Documents/slam_devel/include/eigen3 -DEIGEN_INCLUDE_DIR=$HOME/Documents/slam_devel/include/eigen3 # for Ubuntu 16

# Clone the repository and its dependencies.
cd src

git clone --recursive https://bitbucket.org:JzHuai0108/swift_vio.git

wstool init
wstool merge swift_vio/sliding_window_estimator/dependencies.rosinstall
wstool merge swift_vio/loop_closure/dependencies.rosinstall
wstool update -j 8

catkin build sliding_window_estimator -j4 --cmake-args \
  -DCeres_DIR=$HOME/Documents/slam_devel/lib/cmake/Ceres \
  -DSophus_DIR=$HOME/Documents/slam_devel/lib/cmake/Sophus \
  -DGTSAM_DIR=/home/jhuai/Documents/gtsam_devel/lib/cmake/GTSAM \
  -DGTSAM_UNSTABLE_DIR=/home/jhuai/Documents/gtsam_devel/lib/cmake/GTSAM_UNSTABLE \
  -DBUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DUSE_SYSTEM_OPENGV=OFF

#  -Dopengv_DIR=$HOME/Documents/slam_devel/lib/cmake/opengv-1.0 \ # if opengv is installed beforehand, otherwise pass -DUSE_SYSTEM_OPENGV=OFF
# -Dglog_DIR=$HOME/Documents/slam_devel/lib/cmake/glog # if glog is installed from source

# -DCMAKE_PREFIX_PATH=/opt/ros/$ROS_DISTRO
# -DDO_TIMING=ON
# -DUSE_SANITIZER=Address

```

Explanations of these command arguments are given below.

* BUILD_TESTS=ON
Gtests for OKVIS modules will be built.
Gtests for catkin packages will be built in Section [Run Tests](#run-tests).

* CMAKE_PREFIX_PATH
see [the Development guide](./doc/Develop.md)

* DO_TIMING=ON
Add this cmake flag to compute timing statistics.

* USE_SANITIZER=Address
For debugging tricky memory related bugs, use this option.
But it noticeably slow down the compilation and running.
To disable it, pass USE_SANITIZER=off

* Build Error "ceres-solver/include/ceres/jet.h:887:8: error: ‘ScalarBinaryOpTraits’ is not a class template".
This error arises when the system wide Eigen, e.g., on Ubuntu 16, is incompatible with ceres solver 14.0 
which requires Eigen version >= 3.3.
You need to pass EIGEN_INCLUDE_DIR and EIGEN3_INCLUDE_DIR in building this package and 
give up the point cloud library (PCL)
which depends on system wide Eigen.
The latter has been taken care in [CMakeLists.txt](CMakeLists.txt).
In the end, the workspace should be clear of traces of system wide Eigen. That is, 
no /usr/include/eigen3 should appear when searching in files of the workspace.

### Build Tests for a package
```
catkin build sliding_window_estimator --catkin-make-args tests
```

### Run Tests
You may build and run tests to check the quality of the code.

* To run all tests,
```
catkin build loop_closure_module --catkin-make-args run_tests # or
rosrun loop_closure_module loop_closure_module_test

catkin build sliding_window_estimator --catkin-make-args run_tests # or
rosrun sliding_window_estimator sliding_window_estimator_test
```

* To run selected tests, e.g.,
```
rosrun sliding_window_estimator sliding_window_estimator_test --gtest_filter="*Eigen*"

./build/okvis/okvis_ceres/okvis_ceres_test --gtest_filter="*Imu_BG_BA_MG_TS_MA*"

# test SlidingWindowFilter
rosrun sliding_window_estimator sliding_window_estimator_test --log_dir="/swift_vio_sim/hf_calib_all" \
  --gtest_filter="*SlidingWindowFilter.TrajectoryLabel*" --sim_num_runs=10 \
 --sim_trajectory_label=WavyCircle -v=-1

```

* To test RPGO,
```
cd swift_vio_ws/build/Kimera-RPGO
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
make check
```

* To run integration tests,
Download the evaluation workhorse [rpg_trajectory_evaluation](git@github.com:JzHuai0108/rpg_trajectory_evaluation.git), 
and install its dependencies,
```
pip2 install --upgrade pyyaml
pip2 install numpy matplotlib colorama ruamel.yaml
```
Because python scripts under evaluation/ directory defaults to use python3, 
you also need to install the python3 counterparts,
```
pip3 install numpy matplotlib colorama pyyaml ruamel.yaml
```

For tests, download the EuRoC dataset.
Then run swift_vio/evaluation/smoke_test.py which tests the estimators with one data session from the EuRoC dataset.
Finally, run swift_vio/evaluation/main_evaluation.py which tests the program with multiple data sessions from the EuRoC dataset.

### Error while loading shared libraries: libmetis-gtsam.so 

cannot open shared object file: No such file or directory

```bash
export LD_LIBRARY_PATH=/home/jhuai/Documents/gtsam_devel/lib:$LD_LIBRARY_PATH
```

## Simulation test

```
roscore  # terminal 1

export SWIFT_VIO_WS=/path/to/swift_vio_ws  # terminal 2
rosrun rviz rviz -d $SWIFT_VIO_WS/src/swift_vio/sliding_window_estimator/config/rviz-sim.rviz use_sim_time:=true

export SWIFT_VIO_WS=/path/to/swift_vio_ws  # terminal 3
source $SWIFT_VIO_WS/devel/setup.bash
# sliding window filter
rosrun sliding_window_estimator swift_vio_simulation $SWIFT_VIO_WS/src/swift_vio/sliding_window_estimator/config/sim/simulation_kswf_mincalib.yaml \
  --log_dir="~/Desktop/simulation/" --sim_landmark_model=1 -v=1

# OkvisEstimator / FixedLagSmoother / RiFixedLagSmoother
rosrun sliding_window_estimator swift_vio_sim_mono --sim_algorithm="FixedLagSmoother" \
  $SWIFT_VIO_WS/src/swift_vio/sliding_window_estimator/config/sim/simulation_okvis_calibrated.yaml \
  --log_dir="~/Desktop/simulation/"

```

## Process Datasets

Download a [EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets), for instance MH_01_easy. 

### Process Data from ROS Topics

In terminal 1,
```
roscore
```

In terminal 2,
```
export SWIFT_VIO_WS=/path/to/swift_vio_ws
rosrun rviz rviz -d $SWIFT_VIO_WS/src/swift_vio/sliding_window_estimator/config/rviz.rviz
```

In terminal 3,
```
export SWIFT_VIO_WS=/path/to/swift_vio_ws
source $SWIFT_VIO_WS/devel/setup.bash
rosrun sliding_window_estimator swift_vio_node $SWIFT_VIO_WS/src/swift_vio/sliding_window_estimator/config/config_fpga_p2_euroc.yaml \
 --dump_output_option=3 --load_input_option=0 --output_dir=$HOME/Desktop/temp
```

In terminal 4
```
rosbag play --pause --start=0.0 --rate=1.0 /path/to/euroc/MH_01_easy.bag /cam0/image_raw:=/camera0 /cam1/image_raw:=/camera1 /imu0:=/imu
```

To run with *monocular* camera input, set *monocular_input: true* in config_fpga_p2_euroc.yaml.

To calibrate the *IMU intrinsic parameters*, use [config_fpga_p2_euroc_calib_Imu.yaml](config/config_fpga_p2_euroc_calib_Imu.yaml) instead of config_fpga_p2_euroc.yaml.

### Process a ROS Bag Synchronously
In terminal 1
```
roscore
```
In terminal 2
```
SWIFT_VIO_WS=/path/to/swift_vio_ws
rosrun rviz rviz -d $SWIFT_VIO_WS/src/swift_vio/sliding_window_estimator/config/rviz.rviz
```
In terminal 3
```
SWIFT_VIO_WS=/path/to/swift_vio_ws
cd $SWIFT_VIO_WS
source devel/setup.bash
rosrun sliding_window_estimator swift_vio_node_synchronous $SWIFT_VIO_WS/src/swift_vio/sliding_window_estimator/config/euroc/config_fpga_p2_euroc.yaml \
 --bagname=/path/to/euroc/MH_01_easy.bag --camera_topics="/cam0/image_raw,/cam1/image_raw" --imu_topic="/imu0" \
 --dump_output_option=3 --output_dir=$HOME/Desktop/temp


euroc_yaml=$SWIFT_VIO_WS/src/swift_vio/sliding_window_estimator/config/euroc/config_fpga_p2_euroc.yaml
sed -i "/algorithm/c\    algorithm: OkvisEstimator" $euroc_yaml;
sed -i "/monocular_input/c\monocular_input: true" $euroc_yaml;

rosrun sliding_window_estimator okvis_node_synchronous $euroc_yaml \
 --bagname=/path/to/euroc/MH_01_easy.bag --camera_topics="/cam0/image_raw,/cam1/image_raw" --imu_topic="/imu0" \
 --output_dir=$HOME/Desktop/temp
sed -i "/algorithm/c\    algorithm: SlidingWindowFilter" $euroc_yaml; # reset
sed -i "/monocular_input/c\monocular_input: false" $euroc_yaml; # reset

```
These commands are wrapped in the [launch file](./launch/swift_vio_node_rosbag.launch) which can be invoked by
```
cd $SWIFT_VIO_WS
source devel/setup.bash
roslaunch sliding_window_estimator swift_vio_node_rosbag.launch bag_file:=/path/to/euroc/MH_01_easy.bag
```
Note that you have to pass the proper ROS parameters by reading the launch file, see also 
[a bash script](./tools/bash/process_rosbag.sh).

### Process Smartphone Data
You may use the [MarsLogger program](https://github.com/OSUPCVLab/mobile-ar-sensor-logger) to collect visual inertial data,
then convert these data into a ROS bag, and process the rosbag with swift_vio. See more instructions at [here](https://github.com/OSUPCVLab/mobile-ar-sensor-logger/wiki).

## Input and Output Description

### Parameter Description
The parameters are divided into two groups, those that depend on the operating system and the file system, 
and those that are specific to the program.
Parameters belonging to the first group are typically passed through command line gflags.
Parameters of the second group are typically configured through yaml.

**List of parameters through command line**
|  Command line arguments | Description | Default |
|---|---|---|
|  load_input_option |  0 subscribe to rostopics, 1 load video and IMU csv |  1 |
| dump_output_option | 0 only publish to rostopics, 1 also save nav states to csv, 2, also save nav states and extrinsic parameters, 3 also save nav states and all calibration parameters to csv. | 3 |
| use_Iekf | true iterated EKF, false EKF. For MSCKF only | false |
| max_inc_tol | the maximum infinity norm of the filter correction. 10.0 for outdoors, 2.0 for indoors, though its value should be insensitive | 2.0 |

**Parameters through config yaml**
Remark: set sigma_{param} to zero to disable estimating "param" in filtering methods.

| Configuration parameters | Description | Default |
|---|---|---|
| extrinsic_rep | "P_BC_Q_BC", estimate camera extrinsics; "", fixed camera extrinsics | "" |
| projection_intrinsic_rep | For filters. "FXY_CXY", estimate fx fy cx cy; "FX_CXY", estimate fx, cx, cy with fx = fy; "FX", estimate fx with fx = fy; "", camera projection intrinsic parameters are fixed | "" |
| distortion_type | "radialtangential", "equidistant", "radialtangential8", "fov" | REQUIRED |
| camera_rate | For processing data loaded from a video and an IMU csv, it determines the play speed. In debug mode, half of the normal camera rate is recommended, e.g., 15 Hz | 30 for Release, 15 for Debug |
| image_readout_time | time to read out an entire frame, i.e., the rolling shutter skew. 0 for global shutter, about 0.030 for rolling shutter | 0 |
| sigma_absolute_translation | The standard deviation [m] of the camera extrinsics translation. With OKVIS, e.g. 1.0e-10 for online-calib; With filters, 5e-2 for online extrinsic translation calib, 0 to fix the translation. |  0 |
| sigma_absolute_orientation | The standard deviation [rad] of the camera extrinsics orientation. With OKVIS, e.g. 1.0e-3 for online-calib; With filters with extrinsic_rep == "P_BC_Q_BC", 5e-2 for online extrinsic orientation calib, 0 to fix the extrinsic orientation | 0 |
| sigma_c_relative_translation | For OKVIS only, the std. dev. [m] of the cam. extr. transl. change between frames, e.g. 1.0e-6 for adaptive online calib (not less for numerics) | 0 |
| sigma_c_relative_orientation | For OKVIS only, the std. dev. [rad] of the cam. extr. orient. change between frames, e.g. 1.0e-6 for adaptive online calib (not less for numerics) | 0 |
| timestamp_tolerance | stereo frame out-of-sync tolerance [s] | 0.2/camera_rate, e.g., 0.005 |
| sigma_focal_length | For filters only, set to say 5.0 to estimate focal lengths, set to 0 to fix them | 0.0 |
| sigma_principal_point | For filters only, set to say 5.0 to estimate cx, cy, set to 0 to fix them | 0.0 |
| sigma_distortion | For filters only, set to nonzeros to estimate distortion parameters, set to 0s to fix them. Adapt to distortion types, e.g., [0.01, 0.003, 0.0, 0.0] for "radialtangential", e.g., [0.01] for "fov" | [0.0] * k |
| imageDelay | Used when data is loaded from a video and an IMU csv, image frame time in the video clock - imageDelay = frame time in the IMU clock | 0.0 |
| sigma_td | For filters only, set to say 5e-3 sec to estimate time delay between the camera and IMU, set to 0 to fix the delay | 0.0 |
| sigma_tr | For filters only, set to say 5e-3 sec to estimate the rolling shutter readout time, set to 0 to fix the readout time as image_readout_time | 0.0 |
| sigma_Mg_element | For filters only, set to say 5e-3 to estimate the Tg matrix for gyros, set to 0 to fix the matrix as Identity | 0 |
| sigma_Ts_element | For filters only, set to say 1e-3 to estimate the gravity sensitivity Ts matrix for gyros, set to 0 to fix the matrix as Zero | 0 |
| sigma_Ma_element | For filters only, set to say 5e-3 to estimate the Ta matrix for accelerometers, set to 0 to fix the matrix as Identity | 0 |
| numKeyframes | Number of keyframes in estimation window | 5 |
| numImuFrames | Number of ordinary frames in estimation window, 3 for OKVIS Estimator, 5 for filters. | 3 |
| featureTrackingMethod | 0 BRISK brute force matching with the keyframe scheme, 1 KLT back-to-back framewise tracking, 2 BRISK back-to-back framewise brute force matching | 0 |

### Configuration Files

The /config/ folder contains example configuration files. Please read the [description](#parameter-description)
of parameters in the yaml file, about balancing efficiency and accuracy,
enabling online calibration.

### Coordinate Frames

In terms of coordinate frames and notation, 

* W denotes the World frame (z up), 
* C\_i denotes the i-th camera frame, 
* S denotes the IMU sensor frame,
* B denotes a (user-specified) body frame.

For sliding_window_estimator, S and B are often used interchangeably.

The primary output is the pose T\_WS as a position r\_WS and quaternion 
q\_WS, the velocity in World frame v\_W, gyro biases (b_g),
accelerometer biases (b_a), along with many calibration parameters, 
as well as their standard deviations.

To parse the output csv file when --dump_output_option=3, refer to the [matlab script](./tools/matlab/plotters/drawSwiftVioResult.m).

## Development

If you feel like contributing to the codebase,
you can set up the code cleaning tool linter,
and the QtCreator develop environment as described in [the doc](doc/Develop.md).
