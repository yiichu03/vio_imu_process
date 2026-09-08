# IMU 参考生成与比较：当前入口

更新：2026-09-08。[GitHub](https://github.com/yiichu03/vio_imu_process)，被测程序快照 `d851804`。本次交接修订保存说明和 `validation_20260908/` 日志，不改变比较器；版本索引见 [项目交接总览](../PROJECT_HANDOVER.md)。

## 当前检查范围

| 工具 | 输入/用途 | 当前通过项 |
| --- | --- | --- |
| `gtsam_ref_preint_from_txt` | IMU + 配置 → Combined 15D 参考 | 基准生成，不是算法全面验收 |
| `gtsam_ref_orb_preint_from_txt` | IMU + 配置 → 9D 预积分及 bias RW 参考 | 基准生成，不与 Combined 15D 混用 |
| `compare_preint_outputs` | OpenVINS YAML 与 Combined 参考 | 均值增量、Sigma_z、JincBias、协方差合理性 |
| `compare_vinsmono_gtsam` | VINS TXT 与 Combined 参考 | 仅 Sigma_z、JincBias |
| `compare_orbslam3_gtsam` | ORB TXT 与 9D 参考 | 仅 Sigma_z9、JincBias |

三个比较器当前都不检查 `J_s/J_e`。矩阵容差为逐元素 `1e-4 + 0.015*max(abs(a),abs(b))`。主程序位于 `sliding_window_estimator/src/apps/`。

2026-09-08 的 [OpenVINS 日志](validation_20260908/openvins_compare.log)、[ORB 日志](validation_20260908/orbslam3_compare.log)、[修复版 VINS 日志](validation_20260908/vinsmono_core_compare.log) 均通过。前两项重新比较保存输出；VINS 使用本日内部修复后重新导出的结果。

注意：本目录顶层 `vins_preint_pack.txt` 保留旧版本输出，其 bias Jacobian 超差。不应用该文件复现“核心修复版通过”；最新结果在相邻 VINS 工作区，下面命令明确指定它。

## 容器内复现

宿主机进入已有环境：`docker exec -it swift_vio_noetic_dev bash`；若未运行，先 `docker start swift_vio_noetic_dev`。以下为容器内命令，不重建容器或删除构建目录。

```bash
source /opt/ros/noetic/setup.bash
source /ws/devel/setup.bash
cd /ws
# 源码变化后按需编译：catkin build sliding_window_estimator

IMU_CHECK_DATA=/ws/src/swift_vio/imu_data
IMU_CHECK_REF=$(mktemp -d /tmp/imu-reference-XXXXXX)

rosrun sliding_window_estimator gtsam_ref_preint_from_txt \
  "$IMU_CHECK_DATA/imu_data_Tangent_0.txt" \
  "$IMU_CHECK_DATA/cpc_config_Tangent_0.yaml" "$IMU_CHECK_REF/combined"

rosrun sliding_window_estimator gtsam_ref_orb_preint_from_txt \
  "$IMU_CHECK_DATA/imu_data_Tangent_0.txt" \
  "$IMU_CHECK_DATA/cpc_config_Tangent_0.yaml" "$IMU_CHECK_REF/orb"

rosrun sliding_window_estimator compare_preint_outputs \
  --ov_pack_yaml "$IMU_CHECK_DATA/imu_openvins_prop_preint.yaml" \
  --gtsam_all "$IMU_CHECK_REF/combined/gtsam_ref_preint_all.txt"

rosrun sliding_window_estimator compare_orbslam3_gtsam \
  --orb_all "$IMU_CHECK_DATA/orb_preint_pack.txt" \
  --gtsam_all "$IMU_CHECK_REF/orb/gtsam_ref_orb_preint_all.txt"

# 这里引用当前本地修复版，而非中心仓库保存的旧 VINS 输出。
VINS_CORE_DATA=/home/liuyi/projects/project3_imu/vins_mono_kinetic/catkin_ws/src/VINS-Mono/imu_data
rosrun sliding_window_estimator compare_vinsmono_gtsam \
  --vins_all "$VINS_CORE_DATA/validation_20260908_core/vins_preint_core.txt" \
  --gtsam_all "$IMU_CHECK_REF/combined/gtsam_ref_preint_all.txt" \
  --abs_tol 1e-4 --rel_tol 1.5e-2
```

独立下载此仓库的接手者，需要先取得修复后的 VINS 代码并生成结果，再把 `--vins_all` 改成对应路径。具体 VINS 版本见项目交接总览，不要使用旧快照 `3962798` 复现修复版结果。

`propag_by_preint.cpp` 另有 OKVIS/RK4 与 GTSAM 双向转换实验，不属于上面三个比较器的 PASS 汇总；本轮未完整复跑所有分支。旧命令和混合结果见 `OUTDATED_` 文档，仅供追溯。
