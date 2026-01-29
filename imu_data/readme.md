我已经在老师工程里加了这个新可执行。

新增源码：preint_from_openvins_pack.cpp (line 1)

读 OpenVINS 的 imu_prop_pack.yaml，解析 xs/xe/dt/gravity/Phi_15x15/Sigma_15x15_from_zero
做你要求的：Rws = R_GtoI^T，反解 dR/dP/dV（起始 body(S) 系），OV->[dp,dtheta,dv,dbg,dba] 重排，调用 BuildMaps15_Tangent，输出 Sigma_z、JincBias_*，并打印 recon 误差。
输出 txt（空格分隔逐行）到 out_dir：covRK4_from_openvins_pack.txt / jacRK4_from_openvins_pack.txt / Sigma_z_from_openvins_pack.txt / JincBias_bg_ba_rk4.txt / JincBias_ba_bg_rk4.txt
CMake 添加 target：CMakeLists.txt (line 299)

怎么编译/运行
source /opt/ros/noetic/setup.bash
source /ws/devel/setup.bash

cd /ws
catkin clean sliding_window_estimator -y
catkin build sliding_window_estimator
source /ws/devel/setup.bash

rosrun sliding_window_estimator preint_from_openvins_pack \
  /ws/src/swift_vio/imu_data/imu_prop_pack.yaml \
  /ws/src/swift_vio/imu_data


