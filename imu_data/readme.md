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
DDelta checks:
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

J_e_preint checks:
  absTol=0.000100000000 relTol=0.015000000000
  J_e_preint abs+rel (entrywise): PASS

J_s_preint checks:
  absTol=0.000100000000 relTol=0.015000000000
  J_s_preint abs+rel (entrywise): PASS

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

### 使用vins mono内部函数结果：
rosrun sliding_window_estimator compare_vinsmono_gtsam \
  --vins_all  /ws/src/swift_vio/imu_data/vins_preint_pack_analytic.txt \
  --gtsam_all /ws/src/swift_vio/imu_data/gtsam_ref_out/gtsam_ref_preint_all.txt

### 使用有限差分结果：
rosrun sliding_window_estimator compare_vinsmono_gtsam \
  --vins_all  /ws/src/swift_vio/imu_data/vins_preint_pack_fd.txt \
  --gtsam_all /ws/src/swift_vio/imu_data/gtsam_ref_out/gtsam_ref_preint_all.txt

```
[ OK ] dR
[ OK ] dP
[ OK ] dV
[ OK ] DT
[ OK ] Sigma_z (z=[dphi,dp,dv,dba,dbg])
[FAIL] JincBias_ba_bg (rows=[dphi,dp,dv]): max violation at (0,4)
  a=-0.0420799110401397503 b=-0.0483458147461485396 |a-b|=0.00626590370600878938 tol=0.000825187221192228066 (abs=0.000100000000000000005, rel=0.0149999999999999994)
[ OK ] J_e_preint (rows z, cols x=[dp,dtheta,dv,dba,dbg])
[FAIL] J_s_preint (rows z, cols x=[dp,dtheta,dv,dba,dbg]): max violation at (0,13)
  a=0.0420799110401393062 b=0.0483458147461486298 |a-b|=0.00626590370600932367 tol=0.000825187221192229476 (abs=0.000100000000000000005, rel=0.0149999999999999994)
compare_vinsmono_gtsam failed: comparison failed
```

rosrun sliding_window_estimator compare_vinsmono_gtsam \
  --vins_all  /ws/src/swift_vio/imu_data/vins_preint_pack_fd.txt \
  --gtsam_all /ws/src/swift_vio/imu_data/gtsam_ref_out/gtsam_ref_preint_all.txt
```
[ OK ] dR
[ OK ] dP
[ OK ] dV
[ OK ] DT
[ OK ] Sigma_z (z=[dphi,dp,dv,dba,dbg])
[ OK ] JincBias_ba_bg (rows=[dphi,dp,dv])
[ OK ] J_e_preint (rows z, cols x=[dp,dtheta,dv,dba,dbg])
[ OK ] J_s_preint (rows z, cols x=[dp,dtheta,dv,dba,dbg])
```

# ORB-SLAM3 vs GTSAM (9D preint + 6D bias RW)
# 1) 先在 ORB-SLAM3 容器里跑 exporter，得到 orb_preint_pack.txt
# 2) 拷贝到这里：/ws/src/swift_vio/imu_data/orb_preint_pack.txt
#
# 生成对应的 GTSAM 参考输出（注意：这是 9D + biasRW 的 reference，和 Combined 15D 不同）
rosrun sliding_window_estimator gtsam_ref_orb_preint_from_txt \
  /ws/src/swift_vio/imu_data/imu_data_Tangent_0.txt \
  /ws/src/swift_vio/imu_data/cpc_config_Tangent_0.yaml \
  /ws/src/swift_vio/imu_data/gtsam_ref_out_orb

# 对比 ORB vs GTSAM
rosrun sliding_window_estimator compare_orbslam3_gtsam \
  --orb_all  /ws/src/swift_vio/imu_data/orb_preint_pack.txt \
  --gtsam_all /ws/src/swift_vio/imu_data/gtsam_ref_out_orb/gtsam_ref_orb_preint_all.txt
```
[ OK ] dR
[ OK ] dP
[ OK ] dV
[ OK ] DT
[ OK ] Sigma_z9 (z9=[dphi,dp,dv])
[ OK ] JincBias_ba_bg (rows=[dphi,dp,dv])
[ OK ] Sigma_bias_rw (z6=[dba,dbg])
[ OK ] Sigma_z15 (z15=[dphi,dp,dv,dba,dbg])
[ OK ] J_e_preint (rows z, cols x=[dp,dtheta,dv,dba,dbg])
[ OK ] J_s_preint (rows z, cols x=[dp,dtheta,dv,dba,dbg])
```


root@liuyi:/ws# rosrun sliding_window_estimator compare_orbslam3_gtsam \
>   --orb_all  /ws/src/swift_vio/imu_data/orb_preint_pack.txt \
>   --gtsam_all /ws/src/swift_vio/imu_data/gtsam_ref_out_orb/gtsam_ref_orb_preint_all.txt
[FAIL] Sigma_z9 (z9=[dphi,dp,dv]): max violation at (3,2)
  a=2.28544354438781738 b=-0.0768812670671565473 |a-b|=2.36232481145497397 tol=0.0343816531658172608 (abs=0.000100000000000000005, rel=0.0149999999999999994)
[FAIL] JincBias_ba_bg (rows=[dphi,dp,dv]): max violation at (2,4)
  a=-5.28640937805175781 b=0.0306810620263506077 |a-b|=5.31709044007810849 tol=0.0793961406707763689 (abs=0.000100000000000000005, rel=0.0149999999999999994)
compare_orbslam3_gtsam failed: comparison failed
root@liuyi:/ws# rosrun sliding_window_estimator compare_vinsmono_gtsam \
>   --vins_all  /ws/src/swift_vio/imu_data/vins_preint_pack.txt \
>   --gtsam_all /ws/src/swift_vio/imu_data/gtsam_ref_out/gtsam_ref_preint_all.txt
[FAIL] Sigma_z (z=[dphi,dp,dv,dba,dbg]): max violation at (3,3)
  a=657.746155084751877 b=1157.9529903601383 |a-b|=500.206835275386425 tol=17.3693948554020743 (abs=0.000100000000000000005, rel=0.0149999999999999994)
[FAIL] JincBias_ba_bg (rows=[dphi,dp,dv]): max violation at (2,4)
  a=-5.31510162699269983 b=0.0306810620263506077 |a-b|=5.34578268901905052 tol=0.0798265244048904921 (abs=0.000100000000000000005, rel=0.0149999999999999994)
compare_vinsmono_gtsam failed: comparison failed
