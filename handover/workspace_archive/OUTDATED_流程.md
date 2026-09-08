> OUTDATED（2026-09-08）：本文记录 OpenVINS 早期分步导出/转换方案及旧工具名，当前已使用一体化 exporter。当前入口见 [项目交接总览](../../PROJECT_HANDOVER.md)，以下历史分析保留备查。

 (1) Sigma_z（15×15） 和 (2) JincBias（9×6） 与 GTSAM oracle 在阈值内一致；ΔR/Δp/Δv 也在我们设定的“允许离散化差异”的阈值内一致。
 
## 1) 整体流程：从原始 IMU txt 到可对比的 preint 量（两条链路）
你最终比较的是“同一段 IMU、同一套噪声/重力参数”下的两条链路：
 
1. 链路 A（OpenVINS propagation → 转成 GTSAM Tangent preint 量）:
- OpenVINS 里跑 propagation dump：生成 imu_prop_pack.yaml（里面有 xs/xe/dt/g/Phi/Sigma_from_zero）关键实现：dump_imu_propagation.cpp (line 541)（主传播函数）、 (line 569)（Phi/Sigma 累积）、 (line 626)（写 YAML）。
- 在老师工程里把 pack 转成 Tangent preint（Sigma_z/JincBias）：preint_from_openvins_pack.cpp (line 371) 
 
2. 链路 B（GTSAM oracle：直接用 GTSAM Tangent Combined Preintegration 积分 IMU txt）gtsam_oracle_preint_from_txt.cpp (line 331)
 
3. 对比工具把 A 的输出和 B 的输出做数值对比并给 PASS/FAIL：compare_preint_outputs.cpp (line 277)
 

## 2) dump_imu_propagation：OpenVINS 侧到底“输出了什么”，数学含义是什么
你生成的 imu_prop_pack.yaml（示例：imu_prop_pack.yaml (line 17)）包含：
- xs_nominal / xe_nominal：名义状态（R,p,v,bg,ba），其中旋转是 R_GtoI，四元数顺序 [x,y,z,w]（JPL）见 imu_prop_pack.yaml (line 17) 和 imu_prop_pack.yaml (line 25)。
- Phi_15x15：OpenVINS 误差状态的 transition matrix 见 imu_prop_pack.yaml (line 33)；误差状态顺序见 imu_prop_pack.yaml (line 65)。
- Sigma_15x15_from_zero：从零初始协方差开始传播得到的终点误差协方差（只代表区间噪声累积） 见 imu_prop_pack.yaml (line 49)。
- gravity_g：重力向量与符号说明（OpenVINS 内部用 -g 的形式） 见 imu_prop_pack.yaml (line 70)。

在代码里它是怎么得到 Phi 和 Sigma_from_zero 的？
- 用 OpenVINS 的 单步传播接口拿每个小 dt 的 (F, Qd)，然后在工具里累计：
    Phi ← F * Phi
    Sigma ← F * Sigma * F^T + Qd，Sigma 从 0 开始
    对应 dump_imu_propagation.cpp (line 573) 到 (line 585)（关键两行在 (line 582)、 (line 583)）。 
 
bias 默认策略（你之前担心的坑）
- 若 xs_mode=identity 且用户没显式给 --bg/--ba/--xs_yaml，就默认用 config 的 biases.accel：
    dump_imu_propagation.cpp (line 846)（默认逻辑在 (line 847) 到 (line 852)）
       输出文件里也会写 measurement_model 和 xs_bias_used：
    dump_imu_propagation.cpp (line 636)、 (line 637)（对应 YAML 的 imu_prop_pack.yaml (line 4)、 (line 5)）。 
 
 
## 3) preint_from_openvins_pack：核心逻辑、公式、以及“OpenVINS 特有误差定义”怎么处理
入口：preint_from_openvins_pack.cpp (line 371)

它做的事情可以分成 6 步（代码里也基本按 1→6 注释写了）：

### Step 0：读取 pack（含 JPL 四元数 → 旋转矩阵）
- 读 YAML 的 xs_nominal / xe_nominal / gravity / Phi / Sigma：preint_from_openvins_pack.cpp (line 268)（load_pack）
- 重点：把 q_GtoI_xyzw 转成 R_GtoI 时用的是 OpenVINS 的 JPL quaternion 公式，不是 Eigen 默认 Hamilton：preint_from_openvins_pack.cpp (line 254)（quat_xyzw_to_R） 这行公式与 OpenVINS 源码一致：quat_ops.h (line 152) openVINS 明确说明它是 JPL 且 left-multiplicative：JPLQuat.h (line 33)、JPLQuat.h (line 104)

为什么这个点决定了你最早的 ΔR 会“差 3 rad”？
因为如果把 JPL(x,y,z,w) 当 Hamilton(x,y,z,w) 去喂 Eigen，会在 R 的方向/转置上出现系统性错误，导致 Rws 和 dR 都错。
 
### Step 1：把 OpenVINS 的 nominal 旋转方向统一到老师工程的 Rws（body→world）
OpenVINS pack 给的是 R_GtoI（全局 G 到 IMU I），而老师工程（OKVIS/Swift）用 Rws = R_WB（body 到 world）。
所以用：Rws = R_GtoI^T
对应代码：preint_from_openvins_pack.cpp (line 383)
这一步是你“坐标系/方向”最容易被问的点；你可以直接拿 pack 头里写的：rotation_direction: "R_GtoI"（imu_prop_pack.yaml (line 18)）来佐证。

### Step 2：从 (xs, xe, g, dt) 反解出 dR/dP/dV（都在起始 body(S) 系）
目标是得到满足下式的 dR, dP, dV（与要求一致）：
- Rws_e ≈ Rws_s * dR
- v_e ≈ v_s + g*dt + Rws_s * dV
- p_e ≈ p_s + v_s*dt + 0.5*g*dt^2 + Rws_s * dP

直接反解就是：
- dR = Rws_s^T * Rws_e
- dV = Rws_s^T * (v_e - v_s - g*dt)
- dP = Rws_s^T * (p_e - p_s - v_s*dt - 0.5*g*dt^2)

对应实现：
- preint_from_openvins_pack.cpp (line 392)（dR）
- preint_from_openvins_pack.cpp (line 393)（dP）
- preint_from_openvins_pack.cpp (line 394)（dV）

并做 recon 自检（理论上应接近 0）：
- preint_from_openvins_pack.cpp (line 397)（p_recon/v_recon/R_recon）
- preint_from_openvins_pack.cpp (line 403)（打印 recon errors）
你这次输出 ~1e-14 量级，说明这一步是“严格自洽”的。
 
 
### Step 3（最关键）：把 OpenVINS 的 15D error-state（Φ/Σ）变换到老师工程的 OKVIS 15D error-state
pack 里声明了 OpenVINS error-state：
- 顺序：[dtheta, dp, dv, dbg, dba]
- 姿态扰动：left（q_new = dq ⊗ q）
- nominal 旋转：R_GtoI  见 imu_prop_pack.yaml (line 65)
 
而老师工程（BuildMaps15_Tangent）默认的 OKVIS 误差状态顺序是：
- x_okvis = [dp, dtheta, dv, dbg, dba] 并且这里的 dtheta 是与 Rws（body→world）一致的“world frame 表达”。
 
光做 permutation 不够：还需要处理 dtheta 的 frame。我们在代码里明确做了一个 15×15 的线性变换：
- OV：[dtheta_body, dp_world, dv_world, dbg_body, dba_body]
- OKVIS：[dp_world, dtheta_world, dv_world, dbg_body, dba_body]
- 并用 dtheta_world = Rws * dtheta_body
 
对应：
- preint_from_openvins_pack.cpp (line 344)（build_T_ov_to_okvis）
- preint_from_openvins_pack.cpp (line 358)（build_T_okvis_to_ov）

然后对 Φ/Σ 做正确的“坐标变换”：
- Sigma_okvis = T_e * Sigma_ov * T_e^T
- Phi_okvis = T_e * Phi_ov * T_s^{-1}

对应：
- preint_from_openvins_pack.cpp (line 408)（取 T_e / T_s_inv）
- preint_from_openvins_pack.cpp (line 410)（covRK4）
- preint_from_openvins_pack.cpp (line 411)（jacRK4）
 
这里 T_e 用的是 Rws_e，T_s_inv 用的是 Rws_s，因为 “起点/终点的误差坐标系”不一样

OpenVINS 为什么会让 dtheta 更像 body frame？
OpenVINS 的 JPLQuat 文档给了 left-multiplicative 误差：q ≈ [0.5 δθ; 1] ⊗ q_hat，并写成 R ≈ exp(-δθ) R_hat（注意这个 δθ 的定义跟 R 的方向绑定）
见 JPLQuat.h (line 76)（解释）以及 JPLQuat.h (line 104)（update 的 dq ⊗ q）。
 
### Step 4：用老师的 BuildMaps15_Tangent 把 (covRK4, jacRK4) 映射到 GTSAM Tangent preintegration 的 z-space
这里就是“复用老师代码”的核心：我们直接把老师工程 propag_by_preint.cpp 里那套映射（F/G/covG）复制了过来，确保完全一致。
- preint_from_openvins_pack.cpp (line 44)（BuildMaps15_Tangent，文件头也写了“Copied … to keep mapping identical”在 (line 24)）
- 对照老师参考实现：propag_by_preint.cpp (line 143)
 
BuildMaps15_Tangent 的作用可以一句话概括：
- 给定 Rws_s, dR, dP, dV, dt，构造把 GTSAM Combined(Tangent) 的 residual error z 映射到 OKVIS end-state error 的线性关系（以及逆），用于：
  - Sigma_z = cov(z)
  - JincBias = ∂z/∂bias
在 tangent 情况下，它用到了 Logmap(dR) 的右雅可比 Jr（因为 GTSAM tangent 对旋转增量用的是“右扰动 + log/exp”链式）：
- preint_from_openvins_pack.cpp (line 68)（phi = Logmap(dR)）
- preint_from_openvins_pack.cpp (line 69)（rightJacobian）
- preint_from_openvins_pack.cpp (line 70)（rightJacobianInverse）
（老师那份对应：propag_by_preint.cpp (line 172) 到 (line 175)）
 
### Step 5：算出你最终需要的 Sigma_z 和 JincBias
这一步完全照搬老师“RK4 ↔ preintegration 对齐”的推导方式：
- Sigma_z（15×15）：Sigma_z = covG_inv * covRK4 * covG_inv^T
  - covRK4：这里是“把 OpenVINS 的 Sigma_from_zero 变换到 OKVIS error-state 后”的 end-state 协方差
  - covG_inv：把 end-state error 转到 z-space error 的雅可比逆
  - 对应代码：preint_from_openvins_pack.cpp (line 417)
- JincBias（9×6）：JincBias_bg_ba = G_inv(0:9,0:9) * jacRK4(0:9, 9:15)
（取 dp/dtheta/dv 对 bias 的影响，再映射到 z-space）
  - 对应代码：preint_from_openvins_pack.cpp (line 418)
  - 然后再做一次列交换得到 [ba,bg] 版本（便于对齐 gtsam 输出）：preint_from_openvins_pack.cpp (line 423)
 
这一步在老师参考代码里对应的是：
- Sigma_z_rk4 = maps.covG_inv * covRK4 * maps.covG_inv.transpose()：propag_by_preint.cpp (line 606)
- JincBias_bg_ba_rk4 = G9_inv * jacobianRK4.topRightCorner<9,6>()：propag_by_preint.cpp (line 596)
 
### Step 6：输出调试友好的中间量 + 最终量
输出在同一个 txt 的 blocks（便于 diff）：
- covRK4_from_openvins_pack：Step3 的 covRK4（OKVIS error-state 下的 end-state cov）
- jacRK4_from_openvins_pack：Step3 的 jacRK4（OKVIS error-state 下的 transition）
- Sigma_z_from_openvins_pack：最终要喂给 GTSAM Combined 的 z-space cov
- JincBias_*：最终要喂给 Combined 的 bias Jacobian（9×6）
对应输出实现：preint_from_openvins_pack.cpp (line 428)
 
## 4) gtsam_oracle_preint_from_txt：对比基准是怎么来的（公式/实现）
入口：gtsam_oracle_preint_from_txt.cpp (line 331)
它做的事很直接：
- 读 config：重力和噪声密度（连续谱密度形式）gtsam_oracle_preint_from_txt.cpp (line 167)（load_config_yaml）
- 构造 PreintegratedCombinedMeasurementsT<TangentPreintegration> 参数：gtsam_oracle_preint_from_txt.cpp (line 370)（Params、各个 covariance）
- 读 IMU txt，截取区间（含端点插值），并做梯形平均：omega = 0.5*(w0+w1), acc = 0.5*(a0+a1) gtsam_oracle_preint_from_txt.cpp (line 240)（select interval）gtsam_oracle_preint_from_txt.cpp (line 384)（integrate loop，关键在 (line 390)～ (line 393)）
- 最后取 GTSAM 的输出：deltaRij/deltaPij/deltaVij/deltaTij 和 preintMeasCov() gtsam_oracle_preint_from_txt.cpp (line 395)（dR/dP/dV/DT） gtsam_oracle_preint_from_txt.cpp (line 399)（Sigma_z） bias Jacobian 拼 [H_biasAcc, H_biasOmega]（也就是 [ba,bg]）：gtsam_oracle_preint_from_txt.cpp (line 400)
 ==这里不太对，这里应该从老师的代码里保存==
 
 
## 5) compare_preint_outputs：它到底比较了哪两者？怎么比？
入口：compare_preint_outputs.cpp (line 277)
(A) 它比较的“两者”分别是：
- “OV→pack→converter” 的结果（来自 --ov_all）：
  - Sigma_z_from_openvins_pack (15x15)
  - JincBias_ba_bg_rk4 (9x6)（如果没有这个 block，就读 bg_ba 再换列） 解析位置：compare_preint_outputs.cpp (line 315)（Sigma）和 (line 320)（JincBias）
- “GTSAM oracle” 的结果（来自 --gtsam_all）：
  - dR_gtsam, dP_gtsam, dV_gtsam, DT_gtsam
  - Sigma_z_gtsam
  - JincBias_ba_bg_gtsam 解析位置：compare_preint_outputs.cpp (line 329) 到 (line 335)
同时，它还会从 --ov_pack_yaml 重新计算一份 dR_ov/dP_ov/dV_ov（不从 ov_all 读），确保 Δ 的计算和 preint_from_openvins_pack 完全一致：
- compare_preint_outputs.cpp (line 339)（Rws_s/Rws_e）
- compare_preint_outputs.cpp (line 346)（dR_ov）
- compare_preint_outputs.cpp (line 347)（dP_ov）
- compare_preint_outputs.cpp (line 348)（dV_ov）
 
(B) 它用的指标：
- Δ（均值增量）：
  - angle(dR_ov^T * dR_gtsam)：compare_preint_outputs.cpp (line 352)
  - ||dP_ov-dP_gtsam||： (line 353)
  - ||dV_ov-dV_gtsam||： (line 354)
  - |dt_ov-dt_gtsam|： (line 355) PASS/FAIL 阈值：compare_preint_outputs.cpp (line 374)（注意这里角度阈值放到了 5e-3 rad，因为 OpenVINS 的 nominal 是 RK4，oracle 用的是 GTSAM 的离散模型，均值可能有小差异）
- Sigma_z：
  - maxAbs(Sov-Sref) 和 rel = diff/max(1,maxAbs(ref))：compare_preint_outputs.cpp (line 359) 到 (line 362)
  - 阈值：compare_preint_outputs.cpp (line 407)
- JincBias：
  - 同样 maxAbs 和 rel：compare_preint_outputs.cpp (line 363) 到 (line 366)
  - 阈值：compare_preint_outputs.cpp (line 412)
- 额外 sanity：
  - 对称性 maxAbs(S-S^T)：compare_preint_outputs.cpp (line 367)
  - PSD：最小特征值：compare_preint_outputs.cpp (line 369)
返回码：
- PASS → 0，FAIL → 1：compare_preint_outputs.cpp (line 425)
 
## 6) expectNearAbsRel 是什么（以及为什么我们在 compare 里用了“类似精神”的阈值）
expectNearAbsRel 是老师那套测试工具里用的“逐元素 abs+rel”比较：
- 对每个元素 (i,j)：
  - diff = |a-b|
  - scale = max(|a|,|b|)
  - tol = absTol + relTol * scale
  - 断言 diff <= tol
对应实现：CovPropConfig.hpp (line 54)
我们在 compare_preint_outputs 里对 dP/dV 也用了类似的思想（abs + rel*norm）：
- compare_preint_outputs.cpp (line 400) 
 
 
 
 
 
 
 那你能不能检查一下OpenVINS 的 Propagator/预积分代码里是不是已经有一个现成的 public 函数，直接返回“整个区间的累计 Phi(te,ts) 和累计噪声协方差/Q（或 Sigma_e|s）”了呢？

 
你还记得我们这个bias check的结果是什么吗？它是检查什么的，现在代码里这段起到什么作用
