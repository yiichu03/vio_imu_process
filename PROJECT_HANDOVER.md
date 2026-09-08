# IMU 传播与预积分验证：项目交接总览

更新日期：2026-09-08。VINS `f052fb5`、FAST-LIO2 `e487ddd` 的修复、验证源码及实验结果已提交推送；中心 FAST-LIO2 比较器已随 `9cc3f11` 推送。本文同步发布状态及文档入口。工作区根文件是本地入口，中心仓库 `PROJECT_HANDOVER.md` 是用于在线交接的同步副本。

## 目的与总结

我们的目的，是把不同算法中 IMU 的处理结果，转换成同一套 GTSAM 误差定义，再核对它们的协方差和 Jacobian 是否合理、一致。通俗说：同一段 IMU 数据，交给不同实现计算，再把“结果的不确定程度”和“bias 改一点会让结果变多少”翻译成同一种表达，进行对照。

老师要交接的是能追溯、能复现的验证代码、输入、结果和 GitHub 链接，不是只提供算法原仓库，也不是比较完整 SLAM 系统的轨迹排名。现阶段优先把已有工作讲清楚、收尾，不为 FAST-LIO2 另开一轮大规模开发。

| 方法/类别 | 当前核对结论 | 交接状态 |
| --- | --- | --- |
| ORB-SLAM3 | 9D 协方差、bias Jacobian 通过；核心未改 | 已有可交接代码链接 |
| VINS-Mono | 核心修复后，协方差、bias Jacobian、内部导数回归通过 | 修复提交 `f052fb5` 已推送；旧快照保留超差现象 |
| OpenVINS | 保存输出的均值、协方差、bias Jacobian 比较通过 | 已有可交接代码链接 |
| GTSAM | 参考生成器可运行，重新生成的参考矩阵与归档一致 | 作为对照基准交接，不是“验证了整个 GTSAM” |
| OKVIS | 已有 RK4 传播及研究测试代码 | 可交接代码；本轮未完整复跑其全部测试 |
| IMU preint | 已有传播与预积分互相转换的实验代码 | 本文对应中心仓库相关程序，未定位独立同名仓库 |
| FAST-LIO2 | 两处核心修复后，固定输入的协方差、bias Jacobian 及内部回归通过 | `e487ddd` 已推送至个人仓库；原版失败证据保留 |

这里的“通过”仅指列明的测试。早期需求原文还包括因子对起点、终点状态的 `J_s/J_e`，但当前跨库比较器已移除这两项。因此，不能把本表表述为“除 FAST-LIO2 外，所有早期需求都已完整验收”；若老师仍要 `J_s/J_e`，需要单独确认、补齐。原始需求保存在 [历史需求汇编](OUTDATED_vio_imu.md)，归档不等于取消需求。

### 共同实验口径

- 固定输入：`imu_data_Tangent_0.txt` 和 `cpc_config_Tangent_0.yaml`，2000 条 IMU，时间为 0～9.995 秒，采样间隔 0.005 秒。不能仅根据 YAML 中 `rate: 100` 推断实际输入频率。
- 保持输入、bias、重力和噪声配置一致，先处理坐标系、旋转扰动和 bias 顺序差异，再做数值比较。输出 bias 列顺序为 `[ba,bg]`。
- 当前跨库矩阵判据逐元素为 `|a-b| <= 1e-4 + 0.015*max(|a|,|b|)`；VINS/FAST-LIO2 本轮修复没有放宽此阈值。通过不表示与 GTSAM 数值完全相等。
- VINS/OpenVINS/FAST-LIO2 对照 Combined 15D；ORB 对照 9D 预积分加独立 bias random walk 模型，不能混用两类参考协方差。
- 本轮没有进行完整相机/IMU 数据集的轨迹回归，也没有证明所有输入、运动或时长都通过。

## 1. ORB-SLAM3

GitHub：[验证仓库](https://github.com/yiichu03/orbslam3)，被测代码 `f72ef9c`，最新文档修订 `32508ed` 已推送；[独立比较器](https://github.com/yiichu03/vio_imu_process/blob/main/sliding_window_estimator/src/apps/compare_orbslam3_gtsam.cpp)。

主要文件与改动：

- `src/tools/export_orb_preint_pack.cpp`：新增独立导出工具，调用 ORB 内部预积分，将其旋转误差转换为 GTSAM Tangent 表达，再导出协方差与 bias Jacobian。交接复核时修正的是这层转换。
- `CMakeLists.txt`：增加工具编译入口；`src/imu_data/`：保存输入、配置、GTSAM 参考和输出。
- 核心 `src/ImuTypes.cc`、`include/ImuTypes.h` 未作本轮算法修改。

实验与结果：2026-08-31 从源码重新编译、导出并比较，2026-09-08 再比较保存输出；`Sigma_z9` 和 `JincBias` 均通过。9D 均值、6D bias 协方差和拼接的 15D 矩阵虽有导出，但不属于当前比较器的通过项。

需求判断：现阶段协方差/bias Jacobian 的转换验证已满足；不等于全部因子 Jacobian 或完整 ORB 系统通过。[当前说明与命令](https://github.com/yiichu03/orbslam3/blob/32508ed/src/imu_data/README.md)，[复核日志](imu_data/validation_20260908/orbslam3_compare.log)。

## 2. VINS-Mono

GitHub：[核心修复提交 f052fb5](https://github.com/yiichu03/vinsmono/commit/f052fb5) 已推送，包含代码、测试和结果；旧快照 `3962798` 是超差版。[独立比较器](https://github.com/yiichu03/vio_imu_process/blob/main/sliding_window_estimator/src/apps/compare_vinsmono_gtsam.cpp)。

主要文件与改动：

- `vins_estimator/src/factor/integration_base.h`：实际修正内部积分。四元数在旋转加速度前归一化；旋转导数改为与这一步计算一致的解析导数，同时修正位置、速度及噪声的关联项。
- `vins_estimator/src/tools/export_vins_preint_pack.cpp`：直接读取修复后的内部 Jacobian/协方差，只做坐标转换；有限差分仅用于检查，不替换正式输出。
- `vins_estimator/src/tools/test_imu_preintegration.cpp`、`imu_data/run_validation.sh`：新增内部矩阵、合成运动和一键验证；`vins_estimator/CMakeLists.txt` 注册测试程序。

实验与结果：完整 catkin 编译通过。3 种单步工况核对状态/噪声导数与协方差，6 种序列核对全部 9×6 bias Jacobian、协方差和重传播；用 3 种差分步长检查，均通过。固定 IMU 的内置和独立 GTSAM 比较也均通过。

旧版旋转对陀螺 bias 的 3×3 子块有 6 个元素超差；例如 `(0,4)` 的绝对差从约 `0.006266` 降到 `0.000314`，低于原阈值 `0.000825`。这不是把参考答案填进输出，也不是放宽阈值。修复改变了内部计算，因此不能继续写“VINS 核心未改”。

需求判断：修复版已完成现阶段内部预积分收尾验证并推送，尚无完整 VIO 轨迹回归。[修复与复现说明](https://github.com/yiichu03/vinsmono/blob/f052fb5/imu_data/core_preintegration_fix.md)，[内部测试日志](https://github.com/yiichu03/vinsmono/blob/f052fb5/imu_data/validation_20260908_core/internal_cases.log)，[固定输入测试日志](https://github.com/yiichu03/vinsmono/blob/f052fb5/imu_data/validation_20260908_core/core.log)，[独立比较日志](imu_data/validation_20260908/vinsmono_core_compare.log)。

## 3. OpenVINS

GitHub：[验证仓库](https://github.com/yiichu03/openvins)，被测代码 `e9b6877`，最新文档修订 `2dfca41` 已推送；[独立比较器](https://github.com/yiichu03/vio_imu_process/blob/main/sliding_window_estimator/src/apps/compare_preint_outputs.cpp)。

主要文件与改动：`ov_msckf/src/tools/export_imu_preint_pack.cpp` 是主要新增工具，调用原有传播函数，累计状态转移矩阵和协方差，再转换成预积分量；`ov_msckf/cmake/ROS1.cmake`、`ROS2.cmake` 注册工具，`imu_data/` 保存输入和 YAML 输出。不是重写 OpenVINS 的传播核心。

实验与结果：2026-09-08 对已有 `imu_openvins_prop_preint.yaml` 重新运行比较器，均值增量、15D 协方差、9×6 bias Jacobian、对称性和半正定检查均通过。旋转差约 `0.001262 rad`，位置/速度差约 `0.080416`、`0.013736`，均在现有判据内。本次是保存输出的复核，不是重新运行整个 OpenVINS 系统；已验证环境为 ROS1，不能据此声称 ROS2 已测试。

需求判断：现阶段传播到预积分的转换验证已满足，范围限于上述输入和检查项。[当前说明](https://github.com/yiichu03/openvins/blob/2dfca41/imu_data/readme.md)，[实验日志](imu_data/validation_20260908/openvins_compare.log)。

## 4. GTSAM

GitHub：[中心验证仓库](https://github.com/yiichu03/vio_imu_process)，此前被测程序版本 `d851804`；原参考生成器及三个比较器未改，本次另增 FAST-LIO2 比较器，已随 `9cc3f11` 提交推送；[参考生成器与比较器目录](https://github.com/yiichu03/vio_imu_process/tree/main/sliding_window_estimator/src/apps)。

主要文件与改动：新增 `gtsam_ref_preint_from_txt.cpp` 生成 Combined 15D 参考，`gtsam_ref_orb_preint_from_txt.cpp` 生成 ORB 对应的 9D 参考；原三个比较器及本次 `compare_fastlio_gtsam.cpp` 负责逐元素比较。这些是我们交接的外层验证工具，不是我们重新实现的 GTSAM 核心。

实验与结果：2026-09-08 读取同一 IMU 与配置重新生成两类参考，输出文件与归档版的 SHA-256 完全一致；这些参考用于上述三条链路的独立比较。[生成日志与校验记录](imu_data/validation_20260908/README.md)。

需求判断：作为对照基准及复现工具已具备。它不是“GTSAM 整个库测试全部通过”的结论，也不能用同一种协方差模型强行验证所有方法。[当前说明与命令](imu_data/README.md)。

## 5. OKVIS

GitHub：[本工作区采用的 OKVIS 来源仓库](https://github.com/JzHuai0108/okvis)，本地版本 `9f4887c`；[中心验证代码](https://github.com/yiichu03/vio_imu_process/blob/main/sliding_window_estimator/src/apps/propag_by_preint.cpp)。前者是老师已有实现，不是我们新增的独立验证仓库。

主要文件：`swift_vio_ws/src/okvis/okvis_ceres/include/swift_vio/imu/ImuOdometry.h` 和对应 `.cpp` 提供传播；中心 `propag_by_preint.cpp` 调用其中 RK4 传播，与 GTSAM 预积分进行转换/比较。已有 `TestImuOdometry.cpp` 也包含传播相关测试，这些已有核心和测试不能全部算作我们新增。

实验与结果：代码已在工作区接入并用于研究实验，但本轮没有完整复跑 OKVIS 相关测试组，不能只根据源码中有比较语句或旧笔记判定全部通过。

需求判断：代码可定位、可交接；全量验证结论仍待逐项复核。未复核不等于已发现失败。

## 6. IMU preint（传播与预积分互相转换实验）

GitHub：[实验主程序](https://github.com/yiichu03/vio_imu_process/blob/main/sliding_window_estimator/src/apps/propag_by_preint.cpp)，[已有预积分测试](https://github.com/yiichu03/vio_imu_process/blob/main/sliding_window_estimator/test/gtsam/TestGtsamImuFactor.cpp)。

本工作区未定位到独立名为 `imu preint` 的交接仓库，本节按中心仓库中的预积分实验整理，不另猜一个 GitHub 链接。如果老师指的是另一个独立项目，需要再补充对应位置。

主要文件与实验：`propag_by_preint.cpp` 包含“由预积分还原传播”“由传播构造预积分”和 GTSAM 9D 传播比较，支持 Tangent/Manifold 以及零初始协方差等分支；`TestGtsamImuFactor.cpp` 是已有测试入口。它与上面的 OKVIS、GTSAM 复用同一批底层代码，不应重复算成三个独立新系统。

结果与需求判断：转换实验代码已存在，但本轮没有把全部分支和测试组重新跑完，不能标为全面 PASS。现阶段可以交接文件与用途，全量结果另补。

## 7. FAST-LIO2

2026-09-08：用户已批准最小方案、核心修复/重跑及提交推送。验证代码和原版/修复版结果已随 [e487ddd](https://github.com/yiichu03/FAST_LIO/commit/e487ddd) 发布到 [个人 FAST-LIO2 仓库](https://github.com/yiichu03/FAST_LIO)；中心比较器版本为 `9cc3f11`。最新说明见 [FAST-LIO2 当前验证状态](https://github.com/yiichu03/FAST_LIO/blob/main/imu_data/README.md)，[修复版实验记录](https://github.com/yiichu03/FAST_LIO/blob/main/imu_data/validation_20260908_fixed/README.md)；[中心状态副本](handover/FASTLIO2_STATUS.md) 用于在线交接。

主要新增文件为 `src/tools/export_fastlio_preint_pack.cpp`、`imu_validation_common.hpp`、`test_fastlio_imu_propagation.cpp` 和独立 CMake；`esekfom.hpp` 增加读取实际离散 F 的只读接口，并修复两处 `scalar_type(1/2)` 为 `scalar_type(0.5)`。整数除法原来先得到 0，导致误差传播应有的旋转被漏掉。修复改变真实内部 F/协方差，不能称为“核心未改”。中心新增 `compare_fastlio_gtsam.cpp` 和 14 项比较器控制测试。原有 Livox 驱动适配保留，不混为新增验证成果。

实验与结果：原版固定 IMU 对照中，协方差 119 个、bias Jacobian 26 个元素超差；修复后两项均为 0 个超差。[原版失败证据](https://github.com/yiichu03/FAST_LIO/blob/main/imu_data/validation_20260908/README.md) 独立保留。输入与参考文件哈希一致，阈值仍为 `abs=1e-4, rel=1.5e-2`。有限差分只用于检查，不替换正式解析输出。

3 种单步的 F/噪声/协方差、6 种序列及固定输入的完整 bias Jacobian 均以 3 个差分步长检查并通过；另有 S2 几何分支专用回归通过。独立工具、中心比较器及本地已有 Livox 适配环境中的 `fastlio_mapping` 重新编译成功；驱动适配未纳入提交。另从提交 `e487ddd` 的干净源码重新构建独立工具，内部测试及 GTSAM 对照均通过，确认独立 IMU 验证不依赖这些本地改动。CTest 和 14 项比较器控制测试通过；主程序只编译，不是雷达轨迹实验。

需求判断：修复版已满足本项目列明的固定输入 IMU 协方差/bias Jacobian 核对需求，成果已发布至个人仓库。[官方来源](https://github.com/hku-mars/FAST_LIO/tree/7cc4175de6f8ba2edf34bab02a42195b141027e9) 仅说明基线，不包含本次工具和修复。没有开展雷达建图或轨迹回归，也不包含稀疏分支、所有输入条件或 `J_s/J_e` 的验收。

## 文档与版本维护

- 新入口为本文和各仓库当前 README；旧文统一加 `OUTDATED_`，保留原内容并说明被什么取代。规则见 [AGENTS.md](handover/WORKSPACE_AGENTS.md)。
- 本次归档 15 份：根目录 6 份历史笔记；OpenVINS 学习笔记 1 份；中心仓库 3 份；ORB 旧说明 2 份；VINS 旧接口/原型说明 3 份。上游 README、许可证和实验原始数据未按“过时文档”处理。
- FAST-LIO2 实现后另将最初提案归档为中心 `handover/OUTDATED_FASTLIO2_PLAN.md`，用 `FASTLIO2_STATUS.md` 承载当前结果；不改写提案的历史状态。
- VINS `f052fb5`、OpenVINS `2dfca41`、ORB `32508ed`、FAST-LIO2 `e487ddd` 和中心比较器 `9cc3f11` 均已提交并推送；后续文档提交不改变这些被测计算。FAST-LIO2 只保留已有五个驱动适配文件为本地未提交改动，未混入验证成果。
- 根目录不是 Git 仓库。本文同步至中心仓库 `PROJECT_HANDOVER.md`；工作区维护规则和根目录历史笔记同步至中心 `handover/`，保留本地原件。
- 中心仓库根下的 `imu_data/vins_preint_pack.txt` 仍是旧结果，不能用它代表当前修复版。新日志使用 VINS 仓库 `validation_20260908_core/vins_preint_core.txt`，避免混淆。
