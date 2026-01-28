## Correct the loop closure module where the computed relative pose by PnP and that by nonlinear optimization differ by roughly a T_BC.
In the end, use a loop closure module based on ceres solver instead of gtsam which is rather tricky to install, referring to VINS Mono and OpenVINS.

## Implement a global bundle adjustment that refines all parameters.
Refer to colmap block bundle adjustment, and Maplab 2.0.

## Use learning-based feature descriptor instead of brisk for feature description, and matching.

## Split out simulation and BSpline dependencies

## Remove deprecated factors, EpipolarFactor, PapFactors etc.

## There may be negative diagonal elements in the covariance matrix. Are these elements all are very small and
such occurrences are very rare, e.g. for the beginning of a test?
This has been solved by enforcing symmetry.

## if too many observations exist in one image (>height/2), then it is desirable to create a lookup table of [p_WS, q_WS, v_WS] for every few rows. 

## to test okvis_ros with your own data, check the following for best performance: 

keyframeInsertionOverlapThreshold_,
keyframeInsertionMatchingRatioThreshold_,
and the parameters used in setting file (*.yaml), esp, T_SC, IMU specs, timeLimit, camera_rate, imageDelay, detection threshold and octaves and maxNoKeypoints, and outputPath

In my opinion, timeLimit can be set roughly the reciprocal of camera_rate which determines the play speed in Player.cpp
set publishImuPropagatedState false to enable output keyframe states
you may want to decrease the detection threshold increase octaves and maxNoKeypoints to improve tracking continuity.
set output in the local hard drive rather than a USB drive may improve performance.
The IMU specs has a important role in convergence, esp sigma_g_c, you may want to enlarge (e.g. x2) the values from the spec sheet.

## referring to msckf_mono implemented by daniliidis group, 
use all feature tracks to update states at the last frame

## Compare IMU models
### A. MG_TS_MA model

### B. The simplified model is used in Jung et al. Observability Analysis of IMU Intrinsic Parameters,
w_m = N_g * w_b + b_g + n_g
a_m = N_a * a_b + b_a + n_a
where both N_g and N_a are fully populated.
The inverse model is given by
w_calibrated = M_g * w_m + b_g + n_g
a_calibrated = M_a * a_m + b_a + n_a
where M_g = N_g^{-1} and M_a = N_a^{-1}.
This is done for easy observability analysis.

### C. In ICRA 16 Extending Kalibr, for the reference IMU in a bundle of IMUs, its scaled misalignment model is given by
w_m = M_g * C_gyro_i * w_b + M_ga * C_gyro_i * a_b + b_g + n_g
a_m = M_a * C_i_w * (a_w - g_w) + b_a + n_a
where the i frame is the nominal frame of the accelerometer triad, and has an origin at the accelerometer triad intersection;
the b frame is identical to the i frame of the reference IMU;
the gyro frame is the nominal frame of the gyro triad, and assuming to have the same origin as i;
M_g and M_ga are fully populated, and M_a is a lower triangular matrix.

### D. In TUM VI dataset 2018 paper, the IMU model is defined by
w_calibrated = M_g * w_m + b_g
a_calibrated = M_a * a_m + b_a
where M_a is a lower triangular matrix, and M_g is fully populated.

### E. In Schneider 2019 Observability-aware self-calibration of visual and inertial sensors, 
the body frame coincides with the gyro frame, the IMU model is given by
w_m = T_g * w_b + b_g + n_g
a_m = T_a * C_a_g * C_g_w * (a_w - g_w) + b_a + n_a
where a is the accelerometer frame, g is the gyro frame,
and T_g and T_a are upper triangular matrices.
This model is used for observability analysis and degenerate motion identification in Yang 2020 "Online IMU intrinsic calibration: Is it necessary?"


## Design the coding structure for the swift_vio methods adaptive to different camera and IMU models,
Foundation level: IMU camera landmark speedBias parameter blocks, expressed essentially by Eigen::Vectors
used by Optimizer and Filter

Connection level: IMU Model interface, Camera model interface, interfacing between the
rundimentary parameter blocks, and the actual models. It contains the IMU/Camera model type info which can be set by users.
are exposed to user settings, employed by the Algorithm to manage sensor models

Functional class level: the actual IMU/Camera model, IMU bias model, IMU bias + TgTsTa model, IMU bias + Tg Ts SM model, Camera projection + distortion model
are exposed to user by documentation

Use a underlying trajectory parameterization to serve poses, and underlying variable parameterization to serve values, sort of like Kalibr
Create factors with Jacobians that can be used for either filtering or optimization

## A mysterious issue
I0911 23:19:01.549983   955 FramewiseDescriptorFrontend.cpp:128] Initialized!
QObject::~QObject: Timers cannot be stopped from another thread
terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
  what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument

The root cause for this issue has nothing to do with the report. It is caused by a seemingly innocuous exit(1) in the codebase.

## The timing entries for waitForOptimization and waitForMatching are often unusually large.

## OKVIS leniently add triangulated landmarks to the estimator.
Some landmarks with small disparity may cause rank deficiency in computing the pose covarinace with ceres solver.
OKVIS addObservation() adds observations to the landmarkMap and as residuals to the ceres problem at the same time.
A better approach is to separate adding observations to feature tracks and adding residuals.

## ThreadedKFVio mock test failed in MockVioBackendInterface even with the github okvis master branch.

## The deadreckoning in okvis and that in swift_vio give different results, see DeadreckoningM and DeadreckoningO.
Answer: Disabling marginalization in OKVIS deadreckoning makes its result similar to swift_vio deadreckoning.

## The progress percentage in building swift_vio jumps back and forth with catkin build.
This effect is not observed with OKVIS_ROS.
An comparison of the CMakeLists.txt between swift_vio and okvis_ros does not reveal suspicious differences.

## On EuRoC MH_04, RI-FLS throws gtsam::IndeterminantLinearSystemException working near a pose variable because of significant drift.

## Will marginalization cause the observable parameters, e.g., calibration parameters, to have wrongly optimistic covariance? 
I believe that marginalization will not cause overconfident covariance for these parameters.
To test, run SlidingWindowFilter in simulation to estimate camera extrinsic parameters, and compute their NEES.

