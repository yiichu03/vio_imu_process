# FAST-LIO2 前后对照证据（2026-09-08）

这些日志从 FAST 仓库逐字复制，原日志和完整输出已随 [e487ddd](https://github.com/yiichu03/FAST_LIO/commit/e487ddd) 推送；本目录随中心交接文档提交保存精简副本。当前结论、文件职责和限制见 [状态说明](../FASTLIO2_STATUS.md)。本地原文件位于 `docker_fastlio2/catkin_ws/src/FAST_LIO/imu_data/`，完整矩阵输出及 `fastlio_mapping` 构建日志也在个人 FAST 仓库；构建主程序时使用的本地 Livox 适配未提交。

| 中心副本 | 原 FAST 目录/文件 | 退出码 |
| --- | --- | --- |
| [original_internal.log](original_internal.log) | `validation_20260908/internal_cases.log` | 1 |
| [original_bias_fd.log](original_bias_fd.log) | `validation_20260908/native_bias_fd.log` | 1 |
| [original_gtsam_compare.log](original_gtsam_compare.log) | `validation_20260908/native_compare.log` | 1 |
| [fixed_internal.log](fixed_internal.log) | `validation_20260908_fixed/internal_cases_extended.log` | 0 |
| [fixed_export.log](fixed_export.log) | `validation_20260908_fixed/export.log` | 0 |
| [fixed_gtsam_compare.log](fixed_gtsam_compare.log) | `validation_20260908_fixed/gtsam_compare.log` | 0 |
| [comparator_controls.log](comparator_controls.log) | `validation_20260908_fixed/comparator_controls.log` | 0 |
| [ctest.log](ctest.log) | `validation_20260908_fixed/ctest.log` | 0 |

原版基线 `7cc4175` 加只读 F getter，不改公式；修复版在此基础上将两处 `scalar_type(1/2)` 改为 `scalar_type(0.5)`。最终内部测试比原版多一个测试专用 S2 转动工况，原有 3 种单步/6 种序列及阈值保持不变；固定输入的导出/比较步骤相同。

固定输入、配置、参考的 SHA-256：

```text
IMU: 62442c9c874afce130a4360980d63e30b1f368d80e3acd41a2664c01f7c386be
配置: 072e1af70d5ef25b491b479e18de0c2376aa90d7e26f744b512ff4b34a3ca728
参考: 55db49a1571e640a78632679306f883a55d3c6fcf6134476a555954a61f4d036
```

判据仍为逐元素 `abs=1e-4, rel=1.5e-2`。Sigma 超差元素数 119→0，Jbias 26→0；不是放宽阈值。有限差分只验证内部解析导数，不作为正式输出。均值仅报告，`J_s/J_e`、稀疏分支与完整雷达系统未验收。

原比较日志中的 ROS 缓存警告未阻止程序完成矩阵检查；后续使用二进制直接运行消除了该警告。日志中的本机路径是溯源信息，跨机器复现时须替换路径。
