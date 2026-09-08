> OUTDATED（2026-09-08）：历史容器操作速记，部分路径/镜像说明与后续交接版本不一致，且不包含当前验证流程。当前说明见 [项目交接总览](../../PROJECT_HANDOVER.md)，以下原文保留备查。

下面按你现在的三个容器（fastlio2 / openvins_ros1 / orbslam3_melodic），分别给出 **进入容器（docker exec）** 和 **重新编译** 的最常用命令（按你当时实际环境/路径）。
Ubuntu 版本:lsb_release -a
ROS 发行版（melodic/noetic 等）:rosversion -d
---

## 1) FAST_LIO（容器名：`fastlio2`，镜像：`kenny0407/marslab_fastlio2:latest`）

### 进入容器

```bash
docker start fastlio2
docker exec -it fastlio2 bash
```

### 编译（源码在挂载目录内）

假设你的工作空间在容器内：
`/home/mars_ugv/docker_ws/catkin_ws`

```bash
docker exec -it fastlio2 bash -lc '
source /opt/ros/noetic/setup.bash

# 如果你用到了 livox_ros_driver2 工作空间（你之前是 ws_livox）
source ~/docker_ws/ws_livox/devel/setup.bash

cd ~/docker_ws/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
'
```

> 如果你只改了 `FAST_LIO` 代码，最稳就是重新跑一次 `catkin_make`。
> 如果报找不到 livox_ros_driver2，再把 `source ~/docker_ws/ws_livox/devel/setup.bash` 打开。

---

## 2) OpenVINS（容器名：`openvins_ros1`，镜像：`ov_ros1_20_04:latest`）

### 进入容器

```bash
docker start openvins_ros1
docker exec -it openvins_ros1 bash
```

### 编译（你当时用的是 catkin tools：`catkin build`）

工作空间在容器内：`/catkin_ws`

```bash
docker exec -it openvins_ros1 bash -lc '
source /opt/ros/noetic/setup.bash
cd /catkin_ws
catkin build
source devel/setup.bash
'
```

> 如果你只改了少量包，也可以用：
> `catkin build <package_name>`（例如 `catkin build ov_msckf`）

---

## 3) ORB-SLAM3（容器名：`orbslam3_melodic`，镜像：`orbslam3_melodic:latest`）

### 进入容器

```bash
docker start orbslam3_melodic
docker exec -it orbslam3_melodic bash
```

### 编译（你当时成功的方式：执行 `build.sh`）

源码目录在容器内：`/workspace/ORB_SLAM3`

```bash
docker exec -it orbslam3_melodic bash -lc '
cd /workspace/ORB_SLAM3
chmod +x build.sh
./build.sh
'
```

> 你之前对 OpenCV 版本检查做过修改（`find_package(OpenCV 3.0)` + 注释 fatal error）。
> 只要这个修改还在仓库里，`./build.sh` 就能一直复现同样的编译结果。

---

## 可选：一条命令查看三个容器是否在跑

```bash
docker ps -a | egrep "fastlio2|openvins_ros1|orbslam3_melodic"
```
