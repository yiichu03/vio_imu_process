文件夹有🔓时候 sudo chown -R $USER:$USER /路径/到/文件夹


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
catkin build slidinsg_window_estimator
source /ws/devel/setup.bash

rosrun sliding_window_estimator preint_from_openvins_pack \
  /ws/src/swift_vio/imu_data/imu_prop_pack.yaml \
  /ws/src/swift_vio/imu_data

rosrun sliding_window_estimator gtsam_ref_preint_from_txt \
  /ws/src/swift_vio/imu_data/imu_data_Tangent_0.txt \
  /ws/src/swift_vio/imu_data/cpc_config_Tangent_0.yaml \
  /ws/src/swift_vio/imu_data/gtsam_ref_out



cd /ws
catkin build sliding_window_estimator
source /ws/devel/setup.bash

# 重新生成 ov_all
rosrun sliding_window_estimator preint_from_openvins_pack \
  /ws/src/swift_vio/imu_data/imu_prop_pack.yaml \
  /ws/src/swift_vio/imu_data

# 再对比
rosrun sliding_window_estimator compare_preint_outputs \
  --ov_pack_yaml /ws/src/swift_vio/imu_data/imu_prop_pack.yaml \
  --ov_all /ws/src/swift_vio/imu_data/preint_from_openvins_pack_all.txt \
  --gtsam_all /ws/src/swift_vio/imu_data/gtsam_ref_out/gtsam_ref_preint_all.txt
