# 2026-09-08 交接复核证据

比较器/参考生成器代码：本仓库 `d851804`，本轮未修改。运行环境：已有 `swift_vio_noetic_dev` 容器、ROS Noetic；命令入口见 [上级 README](../README.md)。本目录作为本次交接修订新增证据保存。

| 日志 | 实际输入与检查 | 结果 |
| --- | --- | --- |
| [openvins_compare.log](openvins_compare.log) | 上级已保存 OpenVINS YAML vs Combined 参考；不是新跑整个 OpenVINS | Overall PASS |
| [orbslam3_compare.log](orbslam3_compare.log) | 上级已保存 ORB TXT vs 9D 参考 | Sigma_z9、JincBias PASS |
| [vinsmono_core_compare.log](vinsmono_core_compare.log) | 相邻 VINS 仓库 `imu_data/validation_20260908_core/vins_preint_core.txt` vs Combined 参考 | Sigma_z、JincBias PASS |
| [gtsam_combined_generate.log](gtsam_combined_generate.log) | 上级 IMU/配置，重新生成 Combined 15D 参考 | 退出码 0，文件与归档完全相同 |
| [gtsam_orb_generate.log](gtsam_orb_generate.log) | 同一输入，重新生成 9D + bias RW 参考 | 退出码 0，文件与归档完全相同 |

三个比较器均退出 0，矩阵阈值为 `abs=1e-4, rel=1.5e-2`，不包含 `J_s/J_e`。VINS 使用基于旧快照 `3962798` 的后续核心修复；其单步/合成序列/有限差分日志保存在 VINS 仓库，不在这里重复生成。修复提交号见 [项目交接总览](../../PROJECT_HANDOVER.md)。

同一文件的归档版与本目录新生成版 SHA-256 分别一致：

```text
gtsam_ref_preint_all.txt
55db49a1571e640a78632679306f883a55d3c6fcf6134476a555954a61f4d036

gtsam_ref_orb_preint_all.txt
b130de900660209ea64459e4d0be3276b3864306d9822388717d2158c1448c23
```

对应新文件：[Combined 参考](gtsam_ref_out/gtsam_ref_preint_all.txt)、[ORB 模型参考](gtsam_ref_out_orb/gtsam_ref_orb_preint_all.txt)。这是重生成一致性证据，不是 GTSAM 所有功能正确性的证明。
