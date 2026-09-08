> OUTDATED（2026-09-08）：旧总结仍列出当前比较器已移除的 J_s/J_e，不能用作当前验收清单。请以 [项目交接总览](../../PROJECT_HANDOVER.md) 为准，以下原文保留备查。

# gtsam参考值

1. Sigma_z_gtsam	
15×15	
残差噪声协方差（[dphi,dp,dv,dba,dbg]）	
决定权重（信息矩阵/whitening）
2. JincBias_ba_bg_gtsam	
9×6	
预积分增量对 bias 的导数（[dphi,dp,dv] wrt [dba,dbg]）	
bias 改变时修正预积分量；也进入残差对 bias 的雅可比块
3. J_s	
15×15	
残差对起点误差状态的雅可比	
线性化系统的左端
4. J_e	
15×15	
残差对终点误差状态的雅可比	
线性化系统的右端
