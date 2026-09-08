> OUTDATED（2026-09-08）：早期开发提示记录，含已替换的工具名和当时建议的阈值；保留用于追溯需求，不代表当前实现或验收结果。当前说明见 [项目交接总览](../../PROJECT_HANDOVER.md)。

在 sliding_window_estimator 工程里新增可执行 gtsam_oracle_preint_from_txt.cpp：
输入：imu_txt 与 config_yaml 与 out_dir（可选 ts/te，默认用 imu 首尾）。
读取 config_yaml：
- gravity: [0,0,-g]
- imu_params.sigma_g_c, sigma_a_c, sigma_gw_c, sigma_aw_c
- biases.gyro, biases.accel（若不存在则默认 0）
    构造：gtsam::PreintegratedCombinedMeasurementsT<gtsam::TangentPreintegration>（tangent，必须与 BuildMaps15_Tangent 一致）
- Params(n_gravity)
- gyroscopeCovariance = sigma_g_c^2 * I
- accelerometerCovariance = sigma_a_c^2 * I
- biasOmegaCovariance = sigma_gw_c^2 * I
- biasAccCovariance = sigma_aw_c^2 * I
- integrationCovariance = 1e-16 * I
- biasAccOmegaInt = 0
- use2ndOrderCoriolis=false
- bias_gtsam 用 ConstantBias(accelBias, gyroBias)，bias 来自 yaml
    读取 IMU txt：每行 t gx gy gz ax ay az（与 OpenVINS dump 同单位），按与老师 propag_by_preint.cpp 相同的方式做区间积分（端点截断/插值 + 梯形平均 omega/acc），调用 integrateMeasurement(acc, omega, dt)。
输出到 out_dir（空格分隔逐行）：
- dR_gtsam.txt（3×3, pim.deltaRij().matrix()）
- dP_gtsam.txt（3×1）
- dV_gtsam.txt（3×1）
- DT_gtsam.txt（scalar）
- Sigma_z_gtsam.txt（15×15, pim.preintMeasCov()）
- JincBias_ba_bg_gtsam.txt（9×6, [pim.preintegrated_H_biasAcc(), pim.preintegrated_H_biasOmega()]）
    代码风格用已有 saveMatrixTxt 一致格式。
    
    
    
    
目标：写一个对比工具 compare_preint_outputs，用来验证 “preint_from_openvins_pack_all.txt” 与 “gtsam_oracle_preint_all.txt” 在 ΔR/Δv/Δp、Sigma_z、JincBias 三个层面一致。

输入参数：
- --ov_pack_yaml <imu_prop_pack.yaml>（用于算 ΔR/Δp/Δv）
- --ov_all <preint_from_openvins_pack_all.txt>
- --gtsam_all <gtsam_oracle_preint_all.txt>

需要解析的 block：
- 从 ov_all 解析：Sigma_z_from_openvins_pack (15x15)、JincBias_ba_bg_rk4 (9x6)（或解析 bg_ba 再做列置换到 ba_bg）
- 从 gtsam_all 解析：dR_gtsam (3x3)、dP_gtsam (3x1)、dV_gtsam (3x1)、DT_gtsam (1x1)、Sigma_z_gtsam (15x15)、JincBias_ba_bg_gtsam (9x6)

ΔR/Δp/Δv 的计算（从 ov_pack_yaml）必须和 preint_from_openvins_pack.cpp 完全一致：
- Rws_s = R_GtoI(xs)^T, Rws_e = R_GtoI(xe)^T
- dR = Rws_s^T * Rws_e
- dP = Rws_s^T * (p_e - p_s - v_s*dt - 0.5*g*dt^2)
- dV = Rws_s^T * (v_e - v_s - g*dt)

输出指标（都打印 maxAbs 和 rel；rel=diff/max(1,maxAbs(ref))）：
- Δ：angle( dR_ov^T * dR_gtsam )（rad），||dP_ov-dP_gtsam||，||dV_ov-dV_gtsam||，|dt_ov-dt_gtsam|
- Sigma_z：maxAbs(Sov-Sref)、rel
- JincBias：maxAbs(Jov-Jref)、rel

额外 sanity checks：
- Sigma_z 对称性：maxAbs(S - S^T)
- Sigma_z PSD：最小特征值（允许轻微负数如 -1e-8）

PASS/FAIL 阈值建议（可做成常量）：
- angle(dR)：< 1e-10 rad
- dP/dV：< 1e-6
- Sigma_z rel：< 1e-3
- JincBias rel：< 1e-3
注意：解析 block 时不要依赖固定行号，要用关键 header 定位，然后按 “(15x15)/(9x6)/(3x3)/(3x1)” 读固定行数的数值行。

最终输出：
- 控制台打印每项指标 + PASS/FAIL
- 返回码：PASS 为 0，FAIL 为 1
