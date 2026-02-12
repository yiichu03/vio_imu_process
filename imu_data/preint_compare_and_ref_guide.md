# Swift VIO `imu_data`：`compare` 与 `gtsam_ref` 五个核心逻辑

**统一第一性原理（先固定再比较）**
- 比较本质：同一段 IMU 数据下，比较“同一残差定义、同一误差状态定义”得到的矩阵是否一致。
- 统一残差顺序：`z = [dphi, dp, dv, dba, dbg]`（15维）。
- 统一状态列顺序：`x = [dp, dtheta, dv, dba, dbg]`（15维）。
- 统一判据：逐元素 `tol = absTol + relTol * max(|ref|, |est|)`，这是本项目 compare 的核心标准。

---

## 1) `compare_preint_outputs.cpp`（OpenVINS YAML vs GTSAM TXT）

### 核心代码
```cpp
const OvPack pack = load_ov_pack_yaml(ov_pack_yaml);
const Eigen::Matrix<double,15,15> Sigma_z_ov = pack.Sigma_z;
const Eigen::Matrix<double,9,6> JincBias_ba_bg_ov = pack.JincBias_ba_bg;

const Eigen::Matrix3d dR_gtsam = parse_block_matrix<3,3>(gtsam_all, "dR_gtsam");
const Eigen::Matrix<double,3,1> dP_gtsam = parse_block_matrix<3,1>(gtsam_all, "dP_gtsam");
const Eigen::Matrix<double,3,1> dV_gtsam = parse_block_matrix<3,1>(gtsam_all, "dV_gtsam");
const Eigen::Matrix<double,9,6> JincBias_ba_bg_gtsam = parse_block_matrix<9,6>(gtsam_all, "JincBias_ba_bg_gtsam");
const PreintFactorJacobians jac_gtsam =
    build_preint_factor_jacobians_local(dR_gtsam, dP_gtsam.col(0), dV_gtsam.col(0), dt_gtsam, JincBias_ba_bg_gtsam);

const AbsRelCheckResult Sigma_chk = absRelCheck(Sigma_z_gtsam, Sigma_z_ov, kAbsTol, kRelTol);
const AbsRelCheckResult J_chk = absRelCheck(JincBias_ba_bg_gtsam, JincBias_ba_bg_ov, kAbsTol, kRelTol);
```

### 简洁解释
- OpenVINS 导出的是 YAML（不是 txt），先解析 `Sigma_z/JincBias/J_s/J_e`。
- GTSAM 参考是 txt，解析 `dR/dP/dV/DT/Sigma_z/JincBias`。
- 用同一套局部模型重建 `jac_gtsam.J_s/J_e`，再做 abs+rel 逐元素比较。

---

## 2) `compare_vinsmono_gtsam.cpp`（VINS TXT vs GTSAM TXT）

### 核心代码
```cpp
const auto vins = readMatrixBlocks(vinsAll);
const auto gtsam = readMatrixBlocks(gtsamAll);

const Eigen::MatrixXd Sigma_z_vins = getBlockOrThrow(vins, "Sigma_z_vins_gtsam", vinsAll);
const Eigen::MatrixXd JincBias_vins = getBlockOrThrow(vins, "JincBias_ba_bg_vins", vinsAll);
const Eigen::MatrixXd J_e_preint_vins = getBlockOrThrow(vins, "J_e_preint_vins", vinsAll);
const Eigen::MatrixXd J_s_preint_vins = getBlockOrThrow(vins, "J_s_preint_vins", vinsAll);

const PreintFactorJacobians jac_gtsam =
    build_preint_factor_jacobians_local(dR_gtsam_m, dP_gtsam_v, dV_gtsam_v, DT_gtsam_v, JincBias_gtsam_m);

ok &= expectNearAbsRel(Sigma_z_vins, Sigma_z_gtsam, absTol, relTol, "Sigma_z");
ok &= expectNearAbsRel(JincBias_vins, JincBias_gtsam, absTol, relTol, "JincBias_ba_bg");
ok &= expectNearAbsRel(J_e_preint_vins, jac_gtsam.J_e, absTol, relTol, "J_e_preint");
ok &= expectNearAbsRel(J_s_preint_vins, jac_gtsam.J_s, absTol, relTol, "J_s_preint");
```

### 简洁解释
- 输入是两个 txt block 文件：VINS exporter 输出 vs GTSAM reference 输出。
- 除了比较 `Sigma_z/JincBias`，现在还比较 `J_s/J_e`。
- `J_s/J_e` 的参考值由 `gtsam_all` 的均值增量和 `JincBias`现场重建，不依赖额外文件。

---

## 3) `compare_orbslam3_gtsam.cpp`（ORB TXT vs GTSAM TXT）

### 核心代码
```cpp
const Eigen::MatrixXd Sigma_z9_orb = getBlockOrThrow(orb, "Sigma_z9_orb", orbAll);
const Eigen::MatrixXd JincBias_orb = getBlockOrThrow(orb, "JincBias_ba_bg_orb", orbAll);
const Eigen::MatrixXd Sigma_bias_rw_orb = getBlockOrThrow(orb, "Sigma_bias_rw_orb", orbAll);
const Eigen::MatrixXd Sigma_z15_orb = getBlockOrThrow(orb, "Sigma_z15_orb", orbAll);
const Eigen::MatrixXd J_e_preint_orb = getBlockOrThrow(orb, "J_e_preint_orb", orbAll);
const Eigen::MatrixXd J_s_preint_orb = getBlockOrThrow(orb, "J_s_preint_orb", orbAll);

ok &= expectNearAbsRel(Sigma_z9_orb, Sigma_z9_gtsam, absTol, relTol, "Sigma_z9");
ok &= expectNearAbsRel(JincBias_orb, JincBias_gtsam, absTol, relTol, "JincBias_ba_bg");
ok &= expectNearAbsRel(Sigma_bias_rw_orb, Sigma_bias_rw_gtsam, absTol, relTol, "Sigma_bias_rw");
ok &= expectNearAbsRel(Sigma_z15_orb, Sigma_z15_gtsam, absTol, relTol, "Sigma_z15");
ok &= expectNearAbsRel(J_e_preint_orb, jac_gtsam.J_e, absTol, relTol, "J_e_preint");
ok &= expectNearAbsRel(J_s_preint_orb, jac_gtsam.J_s, absTol, relTol, "J_s_preint");
```

### 简洁解释
- ORB 路线保留 `9D preint + 6D bias RW`，`Sigma_z15` 是 blockdiag，不强求 combined 全交叉项。
- 对比时既检查 `z9/z6/z15`，也检查 `J_s/J_e`。

---

## 4) `gtsam_ref_preint_from_txt.cpp`（VINS/OpenVINS 对齐参考，Combined 15D）

### 核心代码
```cpp
auto params = std::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(n_gravity);
params->gyroscopeCovariance = (cfg.sigma_g_c * cfg.sigma_g_c) * Eigen::Matrix3d::Identity();
params->accelerometerCovariance = (cfg.sigma_a_c * cfg.sigma_a_c) * Eigen::Matrix3d::Identity();
params->biasOmegaCovariance = (cfg.sigma_gw_c * cfg.sigma_gw_c) * gtsam::I_3x3;
params->biasAccCovariance = (cfg.sigma_aw_c * cfg.sigma_aw_c) * gtsam::I_3x3;

gtsam::PreintegratedCombinedMeasurementsT<gtsam::TangentPreintegration> pim(params, bias_gtsam);
for (...) pim.integrateMeasurement(acc, omega, dt);

out.Sigma_z = pim.preintMeasCov();
out.JincBias_ba_bg.block<9, 3>(0, 0) = pim.preintegrated_H_biasAcc();
out.JincBias_ba_bg.block<9, 3>(0, 3) = pim.preintegrated_H_biasOmega();
```

### 简洁解释
- 这是 `compare_vinsmono_gtsam` 与 `compare_preint_outputs` 的参考生成器。
- 输出 `gtsam_ref_preint_all.txt`，包含 `dR/dP/dV/DT/Sigma_z/JincBias`。

---

## 5) `gtsam_ref_orb_preint_from_txt.cpp`（ORB 对齐参考，9D+6D）

### 核心代码
```cpp
auto params = std::make_shared<gtsam::PreintegratedImuMeasurements::Params>(n_gravity);
gtsam::PreintegratedImuMeasurementsT<gtsam::TangentPreintegration> pim(params, bias_gtsam);
for (...) pim.integrateMeasurement(acc, omega, dt);

out.Sigma_z9 = pim.preintMeasCov();
out.JincBias_ba_bg.block<9, 3>(0, 0) = pim.preintegrated_H_biasAcc();
out.JincBias_ba_bg.block<9, 3>(0, 3) = pim.preintegrated_H_biasOmega();

out.Sigma_bias_rw.block<3,3>(0,0) = (cfg.sigma_aw_c * cfg.sigma_aw_c * out.DT) * Eigen::Matrix3d::Identity();
out.Sigma_bias_rw.block<3,3>(3,3) = (cfg.sigma_gw_c * cfg.sigma_gw_c * out.DT) * Eigen::Matrix3d::Identity();
out.Sigma_z15.block<9,9>(0,0) = out.Sigma_z9;
out.Sigma_z15.block<6,6>(9,9) = out.Sigma_bias_rw;
```

### 简洁解释
- 这是 `compare_orbslam3_gtsam` 的参考生成器。
- 输出 `gtsam_ref_orb_preint_all.txt`，主模型是 `z9 + biasRW(z6)`，并提供 `z15` 的 blockdiag 版本。

---

## 6) 详细版（步骤-代码-公式，一步一块）

### Step 1：先生成 GTSAM 参考（`gtsam_ref_preint_from_txt.cpp` / `gtsam_ref_orb_preint_from_txt.cpp`）
```cpp
gtsam::PreintegratedCombinedMeasurementsT<gtsam::TangentPreintegration> pim(params, bias_gtsam);
for (...) {
  pim.integrateMeasurement(acc, omega, dt);
}
out.Sigma_z = pim.preintMeasCov();
```
对应数学（连续到离散的预积分）：
$$
\Delta R_{k+1} = \Delta R_k \exp\!\left((\omega_k-\hat b_g)\Delta t\right),\quad
\Delta v_{k+1} = \Delta v_k + \Delta R_k(a_k-\hat b_a)\Delta t,
$$
$$
\Delta p_{k+1} = \Delta p_k + \Delta v_k\Delta t + \tfrac12\Delta R_k(a_k-\hat b_a)\Delta t^2.
$$
预积分测量噪声协方差：
$$
\Sigma_z = \mathrm{preintMeasCov()}.
$$

### Step 2：构造 bias 一阶雅可比（9x6）
```cpp
out.JincBias_ba_bg.block<9, 3>(0, 0) = pim.preintegrated_H_biasAcc();
out.JincBias_ba_bg.block<9, 3>(0, 3) = pim.preintegrated_H_biasOmega();
```
对应数学：
$$
J_{\text{inc,bias}}=
\begin{bmatrix}
\frac{\partial z_9}{\partial b_a} & \frac{\partial z_9}{\partial b_g}
\end{bmatrix},\quad
z_9=[d\phi,\;dp,\;dv].
$$

### Step 3：从 `dR,dP,dV,DT,JincBias` 重建因子雅可比 \(J_s,J_e\)（compare 内部）
```cpp
const PreintFactorJacobians jac_gtsam =
  build_preint_factor_jacobians_local(dR_gtsam, dP_gtsam.col(0), dV_gtsam.col(0), dt_gtsam, JincBias_ba_bg_gtsam);
```
代码里的线性化模型可写为：
$$
\delta x_e \approx F\,\delta x_s + G_{9}J_{\text{inc,bias}}\delta b + G\,n,
$$
其中 \(x=[dp,d\theta,dv,dba,dbg]\), \(z=[d\phi,dp,dv,dba,dbg]\)。
因此：
$$
J = F + 
\begin{bmatrix}
G_{9}J_{\text{inc,bias}}\\
0_{6\times6}
\end{bmatrix},\quad
J_e = G^{-1},\quad
J_s = -G^{-1}J.
$$

### Step 4：OpenVINS compare 的输入对齐（YAML vs TXT）
```cpp
const OvPack pack = load_ov_pack_yaml(ov_pack_yaml);
const Eigen::Matrix<double,15,15> Sigma_z_ov = pack.Sigma_z;
const Eigen::Matrix<double,15,15> J_e_preint_ov = pack.J_e_preint;
const Eigen::Matrix<double,15,15> J_s_preint_ov = pack.J_s_preint;
```
对应数学目标：比较的是**同一对象**  
$$
\Sigma_z,\;J_{\text{inc,bias}},\;J_e,\;J_s
$$
而不是比较中间状态协方差 \(P\) 或 \(\Phi\)。

### Step 5：VINS/ORB compare 的统一判断公式
```cpp
const double tol = absTol + relTol * std::max(std::abs(ref), std::abs(est));
const bool ok = std::abs(ref - est) <= tol;
```
对应数学：
$$
\left|a_{ij}-b_{ij}\right| \le \underbrace{\epsilon_{\text{abs}} + \epsilon_{\text{rel}}\max(|a_{ij}|,|b_{ij}|)}_{\text{混合容差}}
$$
这个标准保证了：小量看绝对误差，大量看相对误差。

### Step 6：ORB 9D+6D 参考的特殊点
```cpp
out.Sigma_z15.block<9,9>(0,0) = out.Sigma_z9;
out.Sigma_z15.block<6,6>(9,9) = out.Sigma_bias_rw;
```
对应数学：
$$
\Sigma_{z15} =
\begin{bmatrix}
\Sigma_{z9} & 0 \\
0 & \Sigma_{\text{bias-rw}}
\end{bmatrix},
$$
即当前 ORB 路线是 `blockdiag(z9,z6)`，不引入 combined 15D 的全交叉项。

---

## 7) 通俗理解版本
- 你可以把 `gtsam_ref_*` 当成“标准答案生成器”：给它同一段 IMU，它会吐出一套参考的 `dR/dP/dV/DT/Sigma/J`。
- 三个 `compare` 的工作本质都一样：把“被测结果”和“标准答案”按同一顺序摆好，然后逐元素查误差。
- 误差判断不是死板绝对误差，而是“绝对 + 相对”混合容差：小数值看绝对误差，大数值看相对误差。
- `J_s/J_e` 不是多余项，它们回答的是“起点状态动一点，残差怎么变；终点状态动一点，残差怎么变”。
- OpenVINS/VINS/ORB 只是输入格式和内部中间量不同，compare 层最终都归一到同一套变量定义，所以才能公平比较。
