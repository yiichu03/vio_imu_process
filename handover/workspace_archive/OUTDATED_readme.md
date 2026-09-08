> OUTDATED（2026-09-08）：历史环境搭建笔记，包含旧路径和会重建/删除容器的命令，不作为当前复现操作指南。当前状态见 [项目交接总览](../../PROJECT_HANDOVER.md)。以下原文保留备查。

https://github.com/UZ-SLAMLab/ORB_SLAM3
https://github.com/rpng/open_vins
https://github.com/hku-mars/FAST_LIO

ROS melodic ubuntu 18_04
/home/liuyi/projects/project3_imu/orbslam3_melodic/ORB_SLAM3

Dockerfile_ros1_20_04   ROS Noetic ubuntu 20_04
/home/liuyi/projects/project3_imu/openvins/catkin_ws_ov/src/open_vins

ROS Noetic ubuntu 20_04
/home/liuyi/projects/project3_imu/docker_fastlio2/catkin_ws/src/FAST_LIO

# FAST_LIO
确认你的宿主机环境：
```
lsb_release -a
echo "session=$XDG_SESSION_TYPE  display=$DISPLAY"
docker --version
docker ps
```
启动脚本（宿主机执行）
```
mkdir -p /home/liuyi/projects/project3_imu/docker_fastlio2
python3 - <<'PY'
from pathlib import Path

p = Path("/home/liuyi/projects/project3_imu/docker_fastlio2/run_fastlio2.sh")
p.parent.mkdir(parents=True, exist_ok=True)

content = r"""#!/usr/bin/env bash
set -euo pipefail

HOST_DIR="/home/liuyi/projects/project3_imu/docker_fastlio2"
CONTAINER_NAME="fastlio2"

mkdir -p "$HOST_DIR"

# GUI (rviz) 用
xhost +local:

# 若存在同名容器，先删掉
docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true

docker run -itd \
  --name="$CONTAINER_NAME" \
  --user mars_ugv \
  --network host \
  --ipc=host \
  -v "$HOST_DIR":/home/mars_ugv/docker_ws \
  --privileged \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/etc/localtime:/etc/localtime:ro" \
  -v /dev/bus/usb:/dev/bus/usb \
  --device=/dev/dri \
  --group-add video \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --env="DISPLAY=$DISPLAY" \
  kenny0407/marslab_fastlio2:latest \
  /bin/bash
"""
p.write_text(content)
print("written:", p)
PY
```
然后
```
chmod +x /home/liuyi/projects/project3_imu/docker_fastlio2/run_fastlio2.sh
/home/liuyi/projects/project3_imu/docker_fastlio2/run_fastlio2.sh
```
运行后确认容器是否起来
```
docker ps
```
可以看到名为 fastlio2 的容器在跑，继续下一步进入容器：
```
docker exec -it fastlio2 bash
```
进入后确认 ROS 版本和挂载权限：
```
whoami
rosversion -d || true
ls -la /home/mars_ugv/docker_ws
```
路径修改点（相对 README）
只改了 -v 的宿主机路径：
- README：-v /home/$USER/docker_ws:/home/mars_ugv/docker_ws
- 你现在：-v /home/liuyi/projects/project3_imu/docker_fastlio2:/home/mars_ugv/docker_ws
容器内挂载点仍是 /home/mars_ugv/docker_ws（保持不变，避免权限坑）。

确定容器内用户是mars_ugv和readme脚本一致，ROS发行版是noetic,/home/mars_ugv/docker_ws 已经正确挂载到宿主机目录，并且可写（权限是 mars_ugv:mars_ugv）,下面进入“编译 FAST_LIO2（源码）”步骤。

### 1) 先确认容器里有 catkin 工具（容器内执行）
```
which catkin_make
```
如果能输出路径（例如 /usr/bin/catkin_make），继续下一步。
### 2) 在挂载目录里建 catkin 工作空间并拉源码（容器内执行）
这样源码和改动会永久留在宿主机 /home/liuyi/projects/project3_imu/docker_fastlio2/
```
mkdir -p /home/mars_ugv/docker_ws/catkin_ws/src
cd /home/mars_ugv/docker_ws/catkin_ws/src

git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init
```
### 3) 检查 livox_ros_driver 是否已存在（容器内执行）
README 提醒“build 前要 source livox_ros_driver”。你这个镜像有可能已经包含。
执行：
```
source /opt/ros/noetic/setup.bash
rospack find livox_ros_driver || echo "livox_ros_driver NOT found"
```
把输出贴我一下。两种情况：
- 找到了：继续下一步直接编译 FAST_LIO
- 没找到：把 livox_ros_driver 也放进同一个 catkin_ws/src 一起编（最符合 README）

我的情况是“没找到”。
> 你现在这个容器是 ROS Noetic，而 FAST_LIO 依赖的 livox_ros_driver 在官方说明里主要面向 ROS1 indigo/kinetic/melodic，所以你看到 rospack find livox_ros_driver 为 NOT found 是正常的。在 Noetic 下，Livox 官方更推荐装 livox_ros_driver2 + Livox-SDK2。下面给你一套“在当前 Noetic 容器里把 FAST_LIO2 编译起来”的做法：装 driver2，然后把 FAST_LIO 里对 livox_ros_driver/CustomMsg 的引用改成 livox_ros_driver2/CustomMsg。

目录建议（都放在你挂载的 ~/docker_ws 里，保证持久化）
你现在挂载的是 /home/mars_ugv/docker_ws（对应宿主机目录），建议用这三个子目录：
- ~/docker_ws/deps/：放 Livox-SDK2 安装产物（持久化）
- ~/docker_ws/ws_livox/：编译 livox_ros_driver2 的工作空间（持久化）
- ~/docker_ws/catkin_ws/：你现在的 FAST_LIO 工作空间（已存在）
只要这些都在 docker_ws 下，你删容器也不会丢。
### Step 1：装 Livox-SDK2（不走 sudo，装到挂载目录里）
在容器里执行（mars_ugv@...）：
```
cd ~/docker_ws
mkdir -p deps
cd deps

# 1) 拉 SDK2
git clone https://github.com/Livox-SDK/Livox-SDK2.git

# 2) 编译并安装到挂载目录（持久化）
cd Livox-SDK2
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/docker_ws/deps/livox_sdk2
make -j"$(nproc)"
make install

# 3) 配好环境变量（当前 shell 生效）
export CMAKE_PREFIX_PATH=$HOME/docker_ws/deps/livox_sdk2:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$HOME/docker_ws/deps/livox_sdk2/lib:$LD_LIBRARY_PATH

# 4) 快速确认库是否在
ls -la $HOME/docker_ws/deps/livox_sdk2/lib | head
```
### Step 2：编译 livox_ros_driver2（按官方要求用 build.sh）
livox_ros_driver2 官方明确建议：Noetic 下用 ./build.sh ROS1，并且仓库要在 [work_space]/src/ 结构里。
```
cd ~/docker_ws
mkdir -p ws_livox/src
cd ws_livox/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git

# 按官方步骤 build（ROS1/Noetic）
source /opt/ros/noetic/setup.bash
export CMAKE_PREFIX_PATH=$HOME/docker_ws/deps/livox_sdk2:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$HOME/docker_ws/deps/livox_sdk2/lib:$LD_LIBRARY_PATH

cd ~/docker_ws/ws_livox/src/livox_ros_driver2
./build.sh ROS1

# source 这个工作空间（让 rospack 找到包）
source ../../devel/setup.bash

# 验证
rospack find livox_ros_driver2
```
如果这里报类似 “LIVOX_LIDAR_SDK_LIBRARY NOTFOUND”，优先把报错全文贴我（通常就是 SDK2 没被 cmake 找到，继续调整 CMAKE_PREFIX_PATH/LD_LIBRARY_PATH 即可）。

这里确实报错了，运行`./build.sh ROS1`报错了。
> 这个报错的根因是：livox_ros_driver2 需要先安装 Livox-SDK2，否则编译时找不到 SDK2 的头文件 livox_lidar_def.h / livox_lidar_api.h。livox_ros_driver2 的 README 明确要求先 “Build & install the Livox-SDK2”，再 ./build.sh ROS1。你现在就是跳过了 SDK2 安装步骤，所以在 96% 编译阶段报缺头文件。
下面按“最稳妥、最少改动”的方式修复：在容器里把 Livox-SDK2 装到 /usr/local（标准路径，driver2 的构建脚本最容易自动找到），然后重新 build driver2。
### 1) 在容器里安装 Livox-SDK2（用 root 进入容器）
在宿主机执行（开一个 root shell 到容器）：
```
docker exec -it --user root fastlio2 bash
```
在 root shell 里执行：
```
apt-get update
apt-get install -y git cmake build-essential

cd /tmp
rm -rf Livox-SDK2
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j"$(nproc)"
make install
ldconfig

# 验证头文件是否安装到标准位置
ls -la /usr/local/include/livox_lidar_api.h /usr/local/include/livox_lidar_def.h
```
(我make install的时候有警告，但gpt说没关系）
> /usr/local/include 下已经有 livox_lidar_api.h 和 livox_lidar_def.h，/usr/local/lib 下也有 SDK2 的 liblivox_lidar_sdk_shared.so / liblivox_lidar_sdk_static.a，说明 Livox-SDK2 已经成功安装到标准路径，后续 livox_ros_driver2 的编译就应该能找到这些头文件/库了。
看到两个文件路径都存在就 OK。然后 exit 退出 root shell。


### 2) 清理并重新编译 livox_ros_driver2（回到 mars_ugv 容器 shell）

回到你原来的 mars_ugv@... 容器终端里执行：
```
source /opt/ros/noetic/setup.bash

cd ~/docker_ws/ws_livox
rm -rf build devel

cd src/livox_ros_driver2
./build.sh ROS1
```

成功后按 README 的 run 步骤 source 工作空间并验证包存在：
```
source ../../devel/setup.bash
rospack find livox_ros_driver2

```
把 rospack find livox_ros_driver2 的输出贴给gpt。
> rospack find livox_ros_driver2 能找到包，说明 livox_ros_driver2 已经在当前环境可用了。下一步是让 FAST_LIO 在 Noetic 下依赖 livox_ros_driver2（而不是旧的 livox_ros_driver），然后编译你的 catkin_ws。

下面全部在**容器内（mars_ugv）**执行。
### 1) 确认 driver2 的消息类型存在
```
source /opt/ros/noetic/setup.bash
source ~/docker_ws/ws_livox/devel/setup.bash

rosmsg show livox_ros_driver2/CustomMsg | head
```
如果这条命令能输出字段，说明消息生成没问题。
### 2) 修改 FAST_LIO：把 livox_ros_driver 改成 livox_ros_driver2
先看看哪些地方引用了旧包名：
```
cd ~/docker_ws/catkin_ws/src/FAST_LIO
grep -R "livox_ros_driver" -n . | head -n 50
```
这份 grep 里需要改的“会影响编译”的只有这些文件：
- package.xml（依赖包名）
- CMakeLists.txt（catkin 组件依赖）
- src/preprocess.h / src/preprocess.cpp（include + 类型/命名空间）
- src/laserMapping.cpp（include + 类型/命名空间）
README.md 里的出现不用管，不影响编译。
### 1) 一次性替换（容器内执行）
在 ~/docker_ws/catkin_ws/src/FAST_LIO 目录下执行： 
```
cd ~/docker_ws/catkin_ws/src/FAST_LIO

# 1) 替换 include（头文件路径）
grep -rl "<livox_ros_driver/CustomMsg.h>" ./src ./include 2>/dev/null | xargs -r sed -i \
  's|<livox_ros_driver/CustomMsg.h>|<livox_ros_driver2/CustomMsg.h>|g'

# 2) 替换 C++ 命名空间（消息类型的 namespace 跟包名一致）
grep -rl "livox_ros_driver::" ./src ./include 2>/dev/null | xargs -r sed -i \
  's|livox_ros_driver::|livox_ros_driver2::|g'

# 3) 替换 package.xml 里的依赖包名
sed -i 's|>livox_ros_driver<|>livox_ros_driver2<|g' package.xml

# 4) 替换 CMakeLists.txt 里 catkin 组件依赖
sed -i 's|livox_ros_driver|livox_ros_driver2|g' CMakeLists.txt
```
### 2) 再检查一遍（确认只剩 README）
```
grep -R "livox_ros_driver" -n . | head -n 50
```
期望结果：
src/*.cpp、src/*.h、CMakeLists.txt、package.xml 里不再出现 livox_ros_driver
如果还出现，大概率只在 README.md（可忽略）
如果还出现在 src/ 或 CMakeLists.txt，把这条输出贴我
### 3) 编译（关键：先 source ws_livox）
```
source /opt/ros/noetic/setup.bash
source ~/docker_ws/ws_livox/devel/setup.bash

cd ~/docker_ws/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```
验证：
```
rospack find fast_lio
ls -la devel/lib | grep -i fast
```
## 验证编译成功
1. 终端 1（宿主机）
docker exec -it fastlio2 bash
进入后运行：
source /opt/ros/noetic/setup.bash
source ~/docker_ws/catkin_ws/devel/setup.bash
roscore

2. 终端 2（宿主机）
再开一个新的终端窗口，同样执行：
docker exec -it fastlio2 bash
进入后运行：
source /opt/ros/noetic/setup.bash
source ~/docker_ws/catkin_ws/devel/setup.bash
rosrun fast_lio fastlio_mapping
3. 验证（终端 2 或再开终端 3）
docker exec -it fastlio2 bash
source /opt/ros/noetic/setup.bash
source ~/docker_ws/catkin_ws/devel/setup.bash
rosnode list
rostopic list | head
出现了它订阅的 Livox 风格输入 topic：/livox/imu、/livox/lidar，证明启动成功。

## 停止fastlio2 docker容器
全部exit后，在宿主机`docker ps`确认，然后直接停止整个容器（在宿主机）`docker stop fastlio2`

# docker使用：
只看正在运行的容器
docker ps

看所有容器（包括已停止的）
docker ps -a

等价命令：
docker container ls -a

> liuyi@liuyi:~$ docker ps
CONTAINER ID   IMAGE     COMMAND   CREATED   STATUS    PORTS     NAMES
liuyi@liuyi:~$ docker ps -a
CONTAINER ID   IMAGE                               COMMAND                  CREATED             STATUS                     PORTS     NAMES
21b35b726a9b   kenny0407/marslab_fastlio2:latest   "/bin/bash /entrypoi…"   About an hour ago   Exited (0) 2 minutes ago             fastlio2
015a7480be01   my-ros:rao_dev-v1                   "/ros_entrypoint.sh …"   2 months ago        Exited (0) 4 weeks ago               rao_dev


如果你要再次进入/运行 fastlio2
docker start fastlio2
docker exec -it fastlio2 bash
如果你想再次进入/运行 rao_dev
docker start rao_dev
docker exec -it rao_dev bash
（这两个容器都是“停着但还在”，可以随时 start，不需要重新创建。）
结束使用（两种层级）
1. 在容器里输入：exit  只退出当前终端（容器继续运行）
2. 在宿主机执行：docker stop fastlio2   停止容器（推荐你“用完就停”）

## 1) Docker 数据实际存在哪
在宿主机执行：
docker info | grep -E "Docker Root Dir|Storage Driver"
一般会显示类似：
- Docker Root Dir: /var/lib/docker（最常见）
- 如果你装的是 snap 版 docker，也可能是 /var/snap/docker/common/var-lib-docker
## 2) 总共占了多少空间（Docker 角度）
docker system df
想看更详细（每个镜像/容器/volume 的占用）：
docker system df -v
## 3) 你的两个“容器实例”各自占多少（可写层）
docker ps -a --size
你会看到每个容器一行，SIZE 列就是容器可写层占用（不包含镜像层共享部分）。
## 4) 两个镜像各自标称大小
docker image ls
你重点看这两行对应的 SIZE：
- kenny0407/marslab_fastlio2:latest
- my-ros:rao_dev-v1
注意：镜像大小是“虚拟大小”，实际磁盘占用会受层共享影响，所以最准还是看 docker system df -v。
## 5) 从文件系统角度看目录占用（最直观）
如果 docker info 显示 Root Dir 是 /var/lib/docker，你可以看：
sudo du -sh /var/lib/docker
sudo du -sh /var/lib/docker/* | sort -h
> 你之前挂载到容器里的宿主机目录（例如 /home/liuyi/projects/project3_imu/...）不在 Docker Root Dir 里，它占用的是你 home 分区的空间；这部分需要用 du -sh /home/liuyi/projects/project3_imu 另算。



# OpenVINS
你现在 fastlio2 这套是 ROS1 Noetic，为了后续对比实验减少环境分裂，建议 OpenVINS 也先用 ROS1（Dockerfile_ros1_20_04）。官方明确有这个 Dockerfile，并且支持 ROS1 Noetic。
## 1) 宿主机：创建 OpenVINS workspace（建议路径）
```
mkdir -p /home/liuyi/projects/project3_imu/openvins/catkin_ws_ov/src
mkdir -p /home/liuyi/projects/project3_imu/openvins/datasets   # 可选：以后放数据集
cd /home/liuyi/projects/project3_imu/openvins/catkin_ws_ov/src
git clone https://github.com/rpng/open_vins.git
```
说明：后面我们会把 .../catkin_ws_ov 挂载到容器里的 /catkin_ws（官方也是这么做的）。
## 2) 宿主机：build OpenVINS 镜像（ROS1 20.04 / Noetic）
```
cd /home/liuyi/projects/project3_imu/openvins/catkin_ws_ov/src/open_vins
export VERSION=ros1_20_04
docker build -t ov_$VERSION -f Dockerfile_$VERSION .
```
这一步对应官方“用 Dockerfile_ros1_20_04 build 镜像”。
## 3) 宿主机：写一个启动脚本（按你习惯用“命名容器 + docker exec”）
新建脚本：
```
cat > /home/liuyi/projects/project3_imu/openvins/run_openvins_ros1.sh <<'EOF'
#!/bin/bash
set -e

WS_HOST="/home/liuyi/projects/project3_imu/openvins/catkin_ws_ov"
DATA_HOST="/home/liuyi/projects/project3_imu/openvins/datasets"
IMG="ov_ros1_20_04:latest"
NAME="openvins_ros1"

mkdir -p "$WS_HOST" "$DATA_HOST"

# X11 GUI support (rviz)
xhost +local: >/dev/null 2>&1 || true

# remove old container with same name docker rm -f "$NAME" >/dev/null 2>&1 || true

docker run -itd \
  --name="$NAME" \
  --net=host \
  --ipc=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --mount type=bind,source="$WS_HOST",target=/catkin_ws \
  --mount type=bind,source="$DATA_HOST",target=/datasets \
  "$IMG" \
  bash
EOF

chmod +x /home/liuyi/projects/project3_imu/openvins/run_openvins_ros1.sh

```
> 说明：我这里对官方示例做了两点“更贴近你当前工作方式”的调整：用 命名容器 openvins_ros1（方便你 docker start / docker exec），而不是每次 alias ov_docker=... 启一个临时容器。把宿主机 workspace 固定在你指定的 /home/liuyi/projects/project3_imu/openvins/...。bind mount 方式与官方一致：宿主机改动会同步到容器 /catkin_ws。

启动容器 ：
```
/home/liuyi/projects/project3_imu/openvins/run_openvins_ros1.sh
docker exec -it openvins_ros1 bash
```

## 4) 容器内：编译 OpenVINS（ROS1）

进入容器后执行：
```
source /opt/ros/noetic/setup.bash
which catkin || (apt-get update && apt-get install -y python3-catkin-tools)

cd /catkin_ws
catkin build
source devel/setup.bash
```
官方 Docker 指南就是 catkin build + source devel/setup.bash
## 5) 最简“证明编译+可运行”的验证（不需要真实相机/数据集）
1. 终端A：
docker exec -it openvins_ros1 bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
roscore

2. 终端B：
docker exec -it openvins_ros1 bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
roslaunch ov_msckf simulation.launch

3. 然后你可以在终端B或另开终端C检查：
rosnode list
rostopic list | head

或者
```
liuyi@liuyi:~/projects/project3_imu/openvins/catkin_ws_ov/src/open_vins$ docker ps -a | grep openvins_ros1
36940ce08356   ov_ros1_20_04:latest                "/ros_entrypoint.sh …"   21 minutes ago   Up 21 minutes                            openvins_ros1
liuyi@liuyi:~/projects/project3_imu/openvins/catkin_ws_ov/src/open_vins$ docker exec -it openvins_ros1 bash -lc 'ls -la /catkin_ws/src | head'
total 12
drwxrwxr-x  3 user user 4096 Dec 31 12:22 .
drwxrwxr-x  7 user user 4096 Dec 31 12:29 ..
drwxrwxr-x 11 user user 4096 Dec 31 12:22 open_vins
liuyi@liuyi:~/projects/project3_imu/openvins/catkin_ws_ov/src/open_vins$ 
```
## 6) 重要注意：不要同时跑多个 ROS1 容器的 roscore

你现在 fastlio2 也用了 --net=host。如果同时开两个容器都跑 roscore，会端口冲突/话题混杂。建议做法：
- 做 OpenVINS 时：docker stop fastlio2
- 做 FASTLIO2 时：docker stop openvins_ros1

# 如何打开Open VINS和fastlio2
## 1) 现在 OpenVINS 有没有停止：怎么查
在宿主机执行：
```bash
docker ps
```
* 如果列表里 **有** `openvins_ros1`，说明还在运行（Running）。
* 如果列表里 **没有** `openvins_ros1`，再看：
```bash
docker ps -a | grep -E 'openvins_ros1|fastlio2'
```
* `STATUS` 显示 `Exited (...)` 表示已经停止。
* `STATUS` 显示 `Up ...` 表示还在运行。
---

## 2) 如何停止（两种层级）

### A. 停止 OpenVINS / FASTLIO2 内部正在跑的 ROS
在对应容器里运行 `roslaunch` 的终端按：
* `Ctrl-C`
这会停掉 roslaunch/rosrun 进程。
### B. 停止整个容器
在宿主机执行：
```bash
docker stop openvins_ros1
docker stop fastlio2
```
---
## 3) 下次怎么打开 OpenVINS（推荐做法）
你已经有镜像 `ov_ros1_20_04:latest`，并且有绑定工作空间的脚本 `run_openvins_ros1.sh`。
### 方式1：用脚本“重建并启动”（最稳，环境一致）
宿主机：

```bash
/home/liuyi/projects/project3_imu/openvins/run_openvins_ros1.sh
docker exec -it openvins_ros1 bash
```
### 方式2：如果容器还在（只是 stop 了），直接 start
宿主机：

```bash
docker start openvins_ros1
docker exec -it openvins_ros1 bash
```
进入后常规：
```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
```
---
## 4) 下次怎么打开 fastlio2
你的容器名是 `fastlio2`。
### 方式1：直接 start + exec（你已经验证过可行）
宿主机：
```bash
docker start fastlio2
docker exec -it fastlio2 bash
```
进入后：
```bash
source /opt/ros/noetic/setup.bash
source ~/docker_ws/catkin_ws/devel/setup.bash
```
### 方式2：如果你有 run_fastlio2.sh 脚本并且它会 `docker rm -f fastlio2`

那它会“删掉再新建”，适合想要干净环境；不想删就不要在脚本里 `docker rm -f`。
---
## 5) 常用查看/管理命令（宿主机）
```bash
docker ps -a          # 看所有容器及状态
docker logs -n 50 openvins_ros1   # 看最近日志（容器层面）
docker exec -it openvins_ros1 bash
docker exec -it fastlio2 bash
```


# ORB-SLAM3
流程分析（按最省事/可复现）
1. 单独建一个 ORB-SLAM3 Docker 镜像
原因：ORB-SLAM3 需要装/编 Pangolin（通常装到 /usr/local），跟 OpenVINS/FASTLIO2 的环境混在一起容易互相污染；分开镜像可复现、可回滚，也便于你后续改代码做对比实验。
2. 代码放宿主机目录，bind mount 到容器
这样你改 ORB-SLAM3 源码是“永久保存在宿主机”的；容器删了也不丢。
3. 容器里只做编译/运行
ORB-SLAM3 的 build.sh 会：编 Thirdparty → 解压 Vocabulary/ORBvoc.txt.tar.gz → 编 ORB_SLAM3 主库和 Examples。

docker如果是ubuntu20总是报错版本问题，于是换成了ubuntu18

## 1. 宿主机目录规划（建议）

假设你在宿主机使用目录：
mkdir -p /home/liuyi/projects/project3_imu/orbslam3_melodic
cd /home/liuyi/projects/project3_imu/orbslam3_melodic

该目录下放：
Dockerfile
run_orbslam3_melodic.sh
之后容器里会把 ORB_SLAM3 克隆到 /workspace/ORB_SLAM3（映射到宿主机同目录）

## 2. Dockerfile（Ubuntu18 + Melodic + Pangolin）
在宿主机 /home/liuyi/projects/project3_imu/orbslam3_melodic/Dockerfile 写入：
```
FROM osrf/ros:melodic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config \
    libeigen3-dev libopencv-dev \
    python3 python3-numpy \
    libgl1-mesa-dev libegl1-mesa-dev libglew-dev libepoxy-dev \
    libpng-dev libjpeg-dev libtiff-dev libopenexr-dev liblz4-dev libzstd-dev \
    && rm -rf /var/lib/apt/lists/*

# Pangolin（关闭 Python 绑定，减少依赖/报错概率）
RUN cd /tmp && \
    git clone --recursive https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_PANGOLIN_PYTHON=OFF && \
    make -j"$(nproc)" && make install && ldconfig && \
    rm -rf /tmp/Pangolin

WORKDIR /workspace
CMD ["bash"]
```
构建镜像：
```
cd /home/liuyi/projects/project3_imu/orbslam3_melodic
docker build -t orbslam3_melodic:latest .
```
## 3. 启动脚本（命名容器 + 挂载 workspace）

在宿主机写 /home/liuyi/projects/project3_imu/orbslam3_melodic/run_orbslam3_melodic.sh：
```
#!/bin/bash
set -e

WS_HOST="/home/liuyi/projects/project3_imu/orbslam3_melodic"
IMG="orbslam3_melodic:latest"
NAME="orbslam3_melodic"

mkdir -p "$WS_HOST"

# GUI（如果未来要开 Pangolin 窗口）
xhost +local: >/dev/null 2>&1 || true

# 同名容器存在就删掉（不会删代码：代码在宿主机目录）
docker rm -f "$NAME" >/dev/null 2>&1 || true

docker run -itd \
  --name="$NAME" \
  --net=host \
  --ipc=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --mount type=bind,source="$WS_HOST",target=/workspace \
  "$IMG" \
  bash
```

给执行权限并启动：
```
chmod +x /home/liuyi/projects/project3_imu/orbslam3_melodic/run_orbslam3_melodic.sh
/home/liuyi/projects/project3_imu/orbslam3_melodic/run_orbslam3_melodic.sh
docker exec -it orbslam3_melodic bash
```
## 4. 容器内：克隆 ORB_SLAM3 + 子模块（含 safe.directory）

在容器内：
```
cd /workspace
mkdir -p ORB_SLAM3
cd ORB_SLAM3

# 如果目录为空：克隆到当前目录
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git .

# 解决 “dubious ownership”：
git config --global --add safe.directory /workspace/ORB_SLAM3

# 拉子模块
git submodule update --init --recursive
```
（可选）检查版本和你当时一致：
```
g++ --version
cmake --version
pkg-config --modversion opencv
```
## 5. 关键修复：OpenCV 版本检查（Ubuntu18 默认 OpenCV 3.2）
你当时遇到的报错本质是：ORB_SLAM3 的顶层 CMakeLists.txt 强行要求 OpenCV 4.4，而 Melodic/18.04 默认是 OpenCV 3.2。
在容器内执行（位于 /workspace/ORB_SLAM3）：
```
# 允许 OpenCV 3.x
sed -i 's/find_package(OpenCV[[:space:]]\+4\.4/find_package(OpenCV 3.0/' CMakeLists.txt

# 注释掉强制 fatal error（只注释那一行）
sed -i 's/^[[:space:]]*message(FATAL_ERROR "OpenCV > 4\.4 not found\."\))/# &/' CMakeLists.txt

# 确认修改结果
grep -n "find_package(OpenCV" CMakeLists.txt
grep -n 'OpenCV > 4.4 not found' CMakeLists.txt
```
如果 grep 输出类似你当时那样：
```
find_package(OpenCV 3.0)

# message(FATAL_ERROR "OpenCV > 4.4 not found.")
```
就说明改对了。
## 6. 编译 ORB_SLAM3
在容器内：
```
cd /workspace/ORB_SLAM3
chmod +x build.sh
./build.sh
```
## 7. 验证编译成功（你当时的“证明方式”）
在容器内：
```
ls -la /workspace/ORB_SLAM3/lib/libORB_SLAM3.so
ls -la /workspace/ORB_SLAM3/Examples/Monocular/mono_tum
```
出现文件且有可执行权限（-rwx...）即可视为“编译成功”。
你当时看到的那些 warning（例如 -Wreorder、unused variable）不影响编译成功；只有出现 error: 并导致 make 中断才算失败。

## 8. 以后怎么停止 / 再次打开
退出容器 shell（不等于停止容器）
在容器里 exit 只是退出终端会话，容器仍可能在后台运行。
停止容器
宿主机执行：
docker stop orbslam3_melodic
再启动并进入
宿主机执行：
docker start orbslam3_melodic
docker exec -it orbslam3_melodic bash


（只要不删 fastlio2、openvins_ros1 就不会影响那两个环境。）



# 为什么orbslam3_mdelodic的文件夹里面的就是直接orb_slam3，不像其他两个似的有ws文件夹还有和Git仓库并列的build include文件夹

因为 ORB-SLAM3（核心库）不是 ROS/catkin 工程，它本质是一个“独立的 CMake 项目”。所以它的目录形态和 OpenVINS / FAST_LIO 这种 ROS 包不一样。
## 1) OpenVINS / FAST_LIO 为什么有 catkin_ws
它们是 ROS1 工程，标准结构是：
catkin_ws/src/<repo> 放源码
catkin_ws/build、catkin_ws/devel 是 catkin 编译产物（在工作空间根目录生成）
所以你会看到“ws 文件夹 + src/build/devel 并列”。

## 2) ORB-SLAM3 为什么是直接 ORB_SLAM3/ 一堆文件
ORB-SLAM3 官方的构建方式是：
./build.sh（内部调用 CMake + make 编译 Thirdparty 和主库）
产物默认生成在 仓库内部（例如 ORB_SLAM3/build/、ORB_SLAM3/lib/、ORB_SLAM3/Examples/...）
因此它看起来就是一个普通 Git 仓库：CMakeLists.txt / include / src / Examples / Thirdparty ...，不会天然出现 catkin_ws 那套目录。
另外你当时的挂载是把宿主机目录挂到容器 /workspace，然后你在容器里执行了：
cd /workspace/ORB_SLAM3 && git clone ... .
所以宿主机路径 /home/liuyi/projects/project3_imu/orbslam3_melodic/ORB_SLAM3 就直接是仓库根目录。




# swift_vio 编译环境记录
1) 推荐环境：单独一个 swift_vio Noetic 容器（不要塞进 Ubuntu22）

你的 README 明确支持 ROS1 noetic；最稳就是 Ubuntu 20.04 + ROS Noetic 容器。你之后要跟 OpenVINS 对比实验，也能维持 ROS1 统一。

2) 方案 A（推荐）：做一个“带 GTSAM/Sophus 的镜像”，然后在里面 build catkin workspace

好处：以后你删容器也不怕，依赖都在镜像里，可复现。

2.1 写 Dockerfile（swift_vio_noetic.Dockerfile）

在宿主机找个目录新建文件：
```
FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive

# 基础工具 + catkin/wstool/rosdep
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config \
    python3-catkin-tools python3-wstool python3-rosdep \
    libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev libeigen3-dev libsuitesparse-dev \
    libboost-all-dev libtbb-dev \
    libopencv-dev libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# rosdep（容器里 init 可能会报已存在，忽略即可）
RUN rosdep init || true && rosdep update

# Sophus（按 README，需要源码安装）
RUN mkdir -p /opt/src && cd /opt/src && \
    git clone https://github.com/stevenlovegrove/Sophus.git && \
    cd Sophus && mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local && \
    make -j"$(nproc)" && make install

# GTSAM（按 README 固定 commit + 开启 unstable）
RUN cd /opt/src && \
    git clone https://github.com/borglab/gtsam.git --recursive && \
    cd gtsam && git checkout 8c98eefb24f846267119f7f81466dd660f195b06 && \
    mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DGTSAM_BUILD_UNSTABLE=ON -DGTSAM_USE_SYSTEM_EIGEN=ON && \
    make -j"$(nproc)" && make install

# 避免运行时找不到 /usr/local/lib
RUN echo "/usr/local/lib" > /etc/ld.so.conf.d/local.conf && ldconfig

WORKDIR /ws
```

构建镜像：
```
docker build -t swift_vio_noetic -f swift_vio_noetic.Dockerfile .
```
宿主机：准备一个 catkin workspace（把老师项目作为 src 下的一个子目录）

建议新建 workspace（不要直接把 repo 根当 workspace，用起来更稳定）：
```
mkdir -p /home/liuyi/projects/project3_imu/swift_vio_ws/src
```

把你的老师项目放进 src 下面有两种方式：

方式 A（推荐）：软链接（不复制，不破坏你现有目录）
```
ln -s /home/liuyi/projects/project3_imu/JzHuai0108-swift_vio-2aa4b7d13701 \
      /home/liuyi/projects/project3_imu/swift_vio_ws/src/swift_vio
```
这样 swift_vio_ws/src/swift_vio 指向你现有那份代码，不会重复占空间。

2) 宿主机：启动命名容器（bind mount 你的 workspace）
```
NAME=swift_vio_noetic_dev
WS_HOST=/home/liuyi/projects/project3_imu/swift_vio_ws

docker rm -f $NAME 2>/dev/null || true

docker run -itd \
  --name $NAME \
  --net=host \
  --ipc=host \
  -v "$WS_HOST:/ws" \
  swift_vio_noetic \
  bash
```
进入容器：
```
docker exec -it swift_vio_noetic_dev bash
```
现在这个容器名是 swift_vio_noetic_dev（docker ps 里 NAMES 一栏）。

## 1) 停止 Docker 容器（不删除，后面还能进）

在宿主机执行：
```
docker stop swift_vio_noetic_dev
```

验证状态：
```
docker ps          # 这里看不到了（因为只显示 running）
docker ps -a       # 这里能看到，STATUS 会是 Exited
```
2) 下次再启动并进入这个容器
2.1 启动（从 Exited 变回 Up）
```
docker start swift_vio_noetic_dev
```
2.2 进入（拿到一个 shell）
```
docker exec -it swift_vio_noetic_dev bash
```

如果你希望进容器后 ROS 环境自动有（可选），进去后手动：
```
source /opt/ros/noetic/setup.bash
source /ws/devel/setup.bash
```

## 修改代码后的编译：
在 catkin workspace（看起来是 /ws）里用 catkin build。改了某一个 package 以后，通常只需要重编这个 package：
```
cd /ws
catkin build sliding_window_estimator
source /ws/devel/setup.bash
```
如果你改 CMake 后出现“没重新生成”或链接报错，建议加一步清理该包再编：
```
cd /ws
catkin clean sliding_window_estimator -y
catkin build sliding_window_estimator
source /ws/devel/setup.bash
```
说明：source /ws/devel/setup.bash 必须在同一个终端里执行，保证 rosrun 能找到新生成的可执行文件。

关于运行命令：
rosrun sliding_window_estimator imu_file_smoketest --imu_txt=/path/to/imu_data_Tangent_0.txt


# VINS-Mono编译环境记录
仓库：https://github.com/HKUST-Aerial-Robotics/VINS-Mono
容器只提供环境（Ubuntu16.04 + ROS Kinetic + Ceres 1.14）
代码与编译产物落在宿主机（bind mount 到容器），容器删了也不丢
容器名固定，后续用 docker start / docker exec 进入

宿主机目录（当前使用的路径）：/home/liuyi/projects/project3_imu/vins_mono_kinetic/catkin_ws

容器内挂载点：/catkin_ws

容器名：vins_mono_kinetic

## 启动 / 停止 / 再次打开容器（宿主机）
2.1 启动（用你已有脚本）
```
chmod +x /home/liuyi/projects/project3_imu/vins_mono_kinetic/run_vins_mono.sh
/home/liuyi/projects/project3_imu/vins_mono_kinetic/run_vins_mono.sh
```
2.2 停止（不删除）
```
docker stop vins_mono_kinetic
```
2.3 再次启动并进入
```
docker start vins_mono_kinetic
docker exec -it --user "$(id -u)":"$(id -g)" vins_mono_kinetic bash
```
## 进入容器与 Source（容器内）
3.1 进入容器（推荐：用宿主机 UID/GID，避免文件变 root）
```
docker exec -it --user "$(id -u)":"$(id -g)" vins_mono_kinetic bash
```
3.2 必要的 source（容器内）
```
source /opt/ros/kinetic/setup.bash
source /catkin_ws/devel/setup.bash 2>/dev/null || true
```
## 常见权限问题：I have no name! / cannot find name for group ID 1000
4.1 现象

用 --user UID:GID 进入容器时出现：

groups: cannot find name for group ID 1000

I have no name!

4.2 原因

容器内 /etc/passwd / /etc/group 没有对应 UID/GID 的用户/组记录。

4.3 一次性修复（宿主机执行）

只需要做一次。修完后后续 docker exec --user "$(id -u)":"$(id -g)" ... 都不会再出现。

U_ID=$(id -u)
G_ID=$(id -g)
U_NAME=$(id -un)
G_NAME=$(id -gn)

docker exec -it --user root vins_mono_kinetic bash -lc "
set -e
getent group $G_ID >/dev/null || groupadd -g $G_ID $G_NAME
id -u $U_NAME >/dev/null 2>&1 || useradd -m -u $U_ID -g $G_ID -s /bin/bash $U_NAME
"


然后再进入：

docker exec -it --user "$(id -u)":"$(id -g)" vins_mono_kinetic bash

## 拉代码 + 编译（容器内）
5.1 clone（只做一次）
source /opt/ros/kinetic/setup.bash

mkdir -p /catkin_ws/src
cd /catkin_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git

5.2 编译
cd /catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source /catkin_ws/devel/setup.bash

5.3 编译成功的快速验证（容器内）
rospack find vins_estimator
rospack find feature_tracker
rospack find pose_graph

ls -la /catkin_ws/devel/lib/vins_estimator/vins_estimator
ls -la /catkin_ws/devel/lib/feature_tracker/feature_tracker
ls -la /catkin_ws/devel/lib/pose_graph/pose_graph


编译过程中出现 warning（比如 -Wreorder / -Wsign-compare）一般不影响；只有出现 error: 并导致 catkin_make 中断才算失败。

## 修改代码后如何重新编译（宿主机改代码 → 容器内编译）

因为是 bind mount：宿主机 /home/liuyi/.../catkin_ws 会实时映射到容器 /catkin_ws。

6.1 一条命令重新编译（宿主机执行）
docker exec -it vins_mono_kinetic bash -lc \
'source /opt/ros/kinetic/setup.bash && cd /catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release'

6.2 或者进入容器后编译（容器内执行）
source /opt/ros/kinetic/setup.bash
cd /catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source /catkin_ws/devel/setup.bash

## 最简“跑起来”验证（不一定需要 bag）

没有 bag 的情况下，launch 起来后会提示 waiting for image and imu...，这是正常的：说明节点已启动，只是在等输入 topic。

7.1 终端 A：roscore（宿主机新开一个终端）
docker exec -it vins_mono_kinetic bash -lc \
'source /opt/ros/kinetic/setup.bash && source /catkin_ws/devel/setup.bash && roscore'

7.2 终端 B：启动 VINS（宿主机新开一个终端）
docker exec -it vins_mono_kinetic bash -lc \
'source /opt/ros/kinetic/setup.bash && source /catkin_ws/devel/setup.bash && roslaunch vins_estimator euroc.launch'

7.3 终端 C：检查节点与 topic（宿主机新开一个终端）
docker exec -it vins_mono_kinetic bash -lc \
'source /opt/ros/kinetic/setup.bash && source /catkin_ws/devel/setup.bash && rosnode list && rostopic list | head -n 50'

## 容器内查看 Ubuntu / ROS 版本（容器内）
cat /etc/os-release
lsb_release -a 2>/dev/null || true
rosversion -d
