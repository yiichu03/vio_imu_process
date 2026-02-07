文件夹有🔓时候 sudo chown -R $USER:$USER /路径/到/文件夹

docker start swift_vio_noetic_dev
docker exec -it swift_vio_noetic_dev bash
source /opt/ros/noetic/setup.bash
source /ws/devel/setup.bash

新增源码：check_jincbias_from_openvins_yaml.cpp

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

rosrun sliding_window_estimator check_jincbias_from_openvins_yaml \
  /ws/src/swift_vio/imu_data/imu_openvins_prop_preint.yaml

rosrun sliding_window_estimator gtsam_ref_preint_from_txt \
  /ws/src/swift_vio/imu_data/imu_data_Tangent_0.txt \
  /ws/src/swift_vio/imu_data/cpc_config_Tangent_0.yaml \
  /ws/src/swift_vio/imu_data/gtsam_ref_out



cd /ws
catkin build sliding_window_estimator
source /ws/devel/setup.bash

# OpenVINS 会输出一个同时包含 propagation + gtsam tangent preint 的 YAML（从 OpenVINS 容器里拷贝过来）
# /ws/src/swift_vio/imu_data/imu_openvins_prop_preint.yaml

# 对比 OpenVINS vs GTSAM 参考实现
rosrun sliding_window_estimator compare_preint_outputs \
  --ov_pack_yaml /ws/src/swift_vio/imu_data/imu_openvins_prop_preint.yaml \
  --gtsam_all /ws/src/swift_vio/imu_data/gtsam_ref_out/gtsam_ref_preint_all.txt


```
root@liuyi:/ws# rosrun sliding_window_estimator compare_preint_outputs   --ov_pack_yaml /ws/src/swift_vio/imu_data/imu_openvins_prop_preint.yaml   --gtsam_all /ws/src/swift_vio/imu_data/gtsam_ref_out/gtsam_ref_preint_all.txt
Delta checks:
  angle(dR_ov^T*dR_gtsam) = 0.001261670008 rad
  ||dP_ov-dP_gtsam||      = 0.080415920701
  ||dV_ov-dV_gtsam||      = 0.013735819715
  |dt_ov-dt_gtsam|        = 0.000000000000
  rel_dP                  = 0.000175010999
  rel_dV                  = 0.000152794338
  angle(dR) < 5e-3 rad: PASS
  dP abs/rel: PASS
  dV abs/rel: PASS
  dt < 1e-12: PASS

Sigma_z checks:
  absTol=0.000100000000 relTol=0.015000000000
  Sigma_z abs+rel (entrywise): PASS

JincBias checks:
  absTol=0.000100000000 relTol=0.015000000000
  JincBias abs+rel (entrywise): PASS

Sanity checks:
  Sigma_z_ov symmetry maxAbs(S-S^T)   = 0.000000000000
  Sigma_z_gtsam symmetry maxAbs(S-S^T)= 0.000000000003
  Sigma_z_ov min eigen (sym)          = 0.000038391991
  Sigma_z_gtsam min eigen (sym)       = 0.000038393967
  Sigma_z_ov symmetry < 1e-8: PASS
  Sigma_z_gtsam symmetry < 1e-8: PASS
  Sigma_z_ov minEig >= -1e-8: PASS
  Sigma_z_gtsam minEig >= -1e-8: PASS

Overall: PASS

```

# VINS MONO vs GTSAM
rosrun sliding_window_estimator compare_vinsmono_gtsam \
  --vins_all  /ws/src/swift_vio/imu_data/vins_preint_pack.txt \
  --gtsam_all /ws/src/swift_vio/imu_data/gtsam_ref_out/gtsam_ref_preint_all.txt
