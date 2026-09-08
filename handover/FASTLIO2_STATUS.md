# FAST-LIO2：修复后已通过当前 IMU 验证

更新：2026-09-08。用户已批准原方案、修复/重跑及提交推送。FAST-LIO2 验证源码/结果已随 [e487ddd](https://github.com/yiichu03/FAST_LIO/commit/e487ddd) 发布到 [个人仓库](https://github.com/yiichu03/FAST_LIO)；中心比较器已随 `9cc3f11` 推送。本次文档整理已发布结果与版本入口。[原方案](OUTDATED_FASTLIO2_PLAN.md) 已归档。

## 总结

FAST-LIO2 可以完成我们当前的目的：统一 IMU/噪声/初值，读取实际 IMU 传播的协方差、状态转移，转换为 GTSAM 定义后核对。本地原版有一处明确的整数除法问题，不能将其误判为方法客观不可行。最小修复后，内部检查及既有阈值下的跨库矩阵比较全部通过。

## 添加与修改

基线为 [官方 FAST_LIO 7cc4175](https://github.com/hku-mars/FAST_LIO/tree/7cc4175de6f8ba2edf34bab02a42195b141027e9)，此上游链接不包含本次成果。本地仓库位置为工作区的 `docker_fastlio2/catkin_ws/src/FAST_LIO`；成果入口为 [个人 FAST-LIO2 验证说明](https://github.com/yiichu03/FAST_LIO/blob/main/imu_data/README.md)。

- FAST 仓库 `src/tools/`：独立导出器、公共转换/输入工具、内部有限差分回归和 CMake；`imu_data/`：固定输入、配置、一键脚本、前后结果与 README。
- 核心 `include/IKFoM_toolkit/esekfom/esekfom.hpp`：新增实际 F 的只读接口；将 SO3/S2 两处 `scalar_type(1/2)` 改为 `scalar_type(0.5)`。整数除法原来先得到 0，漏掉误差传播应有的旋转；修复影响真实 F/协方差，不是仅更换导出数据。
- 中心新增 [compare_fastlio_gtsam.cpp](../sliding_window_estimator/src/apps/compare_fastlio_gtsam.cpp)、[14 项比较器控制测试](../sliding_window_estimator/src/apps/test_compare_fastlio_gtsam.py) 与 CMake 入口，未修改旧比较器及 GTSAM 参考。
- 原有 Livox 驱动适配保留在本地，未纳入本次提交；根 CMake 仅提交新增验证入口。独立 IMU 验证不依赖驱动适配。

原生 23 维误差转为 `[dphi,dp,dv,dba,dbg]` 的 15 维表达，bias 列 `[ba,bg]`；重力 9.81、外参固定、P 从零开始。Q 按连续噪声密度换算为 `diag(sigma²)/dt`。旋转做右局部误差到 Log 增量转换，bias 协方差采用参考的 `b_start-b_end` 符号。有限差分只检查解析矩阵，不替换正式结果。

## 实验及结果

固定 2000 条 IMU，0～9.995 秒；输入/配置/Combined 15D 参考哈希均未改变。逐元素判据仍为 `1e-4 + 0.015*max(abs(a),abs(b))`。

| 检查 | 原版 | 修复后 |
| --- | --- | --- |
| 内部单步 F/协方差、6 种序列 bias 检查 | 转动工况失败 | 3 种差分步长全部通过 |
| 固定 IMU 内部 bias 差分 | 27 个元素失败 | 0 个失败 |
| GTSAM Sigma15 | 119 个元素超差 | 0 个超差 |
| GTSAM bias9×6 | 26 个元素超差 | 0 个超差 |

S2 测试专用转动模型也通过，覆盖第二处核心修复；真实 FAST-LIO 导出没有改变固定重力模型。独立工具、中心比较器和原 `fastlio_mapping` 主程序均已重新编译成功；主程序构建使用本地已有 Livox 适配环境，该适配未提交，不能据此声称上游驱动环境也已测试。比较器 14 项合成控制测试通过，但不作为算法实验计数。

Sigma/Jbias 最大绝对差约 2.24224/1.18071，结合各元素量级满足原相对/绝对联合判据；“通过”不等于逐位相同。均值仅报告：旋转差约 0.00127145 rad，位置/速度差约 0.237483/0.0332259，不将其写为已验收项。

精简证据同步至 [fastlio2_evidence](fastlio2_evidence/README.md)。完整原始输出、构建日志、复现命令与哈希见 [修复版记录](https://github.com/yiichu03/FAST_LIO/blob/main/imu_data/validation_20260908_fixed/README.md)；[原版失败记录](https://github.com/yiichu03/FAST_LIO/blob/main/imu_data/validation_20260908/README.md) 独立保留。

## 需求覆盖与版本状态

已满足本项目列明的固定 IMU 协方差/bias Jacobian 验证范围，并有内部回归证据。没有开展完整 LiDAR/SLAM 轨迹验证，没有验收 `J_s/J_e`，稀疏 IKFoM 分支未测试；不能扩展为“FAST-LIO2 所有功能正确”。

已按授权发布到现有个人 FAST-LIO2 fork，未向官方上游推送或改动其代码。中心比较器需使用 `9cc3f11` 或后续版本；此前 `7e54059` 不包含新比较器。FAST-LIO2 主代码/结果使用 `e487ddd`，后续文档提交不改变被测计算。
