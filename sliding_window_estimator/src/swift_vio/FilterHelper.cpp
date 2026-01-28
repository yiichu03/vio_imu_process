#include "swift_vio/FilterHelper.hpp"

#include "okvis/ceres/PoseParameterBlock.hpp"
#include <swift_vio/ceres/EuclideanParamBlock.hpp>
#include <swift_vio/IoUtil.hpp>

#include <fstream>

#include <Eigen/QR>

namespace swift_vio {
void FilterHelper::stackJacobianAndResidual(
    const std::vector<Eigen::MatrixXd,
                      Eigen::aligned_allocator<Eigen::MatrixXd>> &vH_o,
    const std::vector<Eigen::Matrix<double, -1, 1>,
                      Eigen::aligned_allocator<Eigen::Matrix<double, -1, 1>>>
        &vr_o,
    const std::vector<Eigen::MatrixXd,
                      Eigen::aligned_allocator<Eigen::MatrixXd>> &vR_o,
    Eigen::MatrixXd *H_o, Eigen::Matrix<double, -1, 1> *r_o, Eigen::MatrixXd *R_o) {
  int startRow = 0;
  int cols = vH_o[0].cols();
  for (size_t jack = 0; jack < vH_o.size(); ++jack) {
    int rows = vH_o[jack].rows();
    H_o->block(startRow, 0, rows, cols) = vH_o[jack];
    r_o->block(startRow, 0, rows, 1) = vr_o[jack];
    R_o->block(startRow, startRow, rows, rows) = vR_o[jack];
    startRow += rows;
  }
}

void FilterHelper::shrinkResidual(const Eigen::MatrixXd& H_o,
                                  const Eigen::MatrixXd& r_o,
                                  const Eigen::MatrixXd& R_o,
                                  Eigen::MatrixXd* T_H,
                                  Eigen::Matrix<double, Eigen::Dynamic, 1>* r_q,
                                  Eigen::MatrixXd* R_q) {
  int Hcols = H_o.cols();
  if (r_o.rows() <= Hcols) {
    *r_q = r_o;
    *T_H = H_o;
    *R_q = R_o;
  } else {  // project H_o, reduce the residual dimension
    // Alternatively, use SPQR instead for computing T_H, refer to
    // https://github.com/KumarRobotics/msckf_vio/blob/master/src/msckf_vio.cpp#L930-L950
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(H_o);
    Eigen::MatrixXd thinQ(Eigen::MatrixXd::Identity(H_o.rows(), H_o.cols()));
    thinQ = qr.householderQ() * thinQ;

    *r_q = thinQ.transpose() * r_o;
    *R_q = thinQ.transpose() * R_o * thinQ;
    Eigen::MatrixXd R = qr.matrixQR().triangularView<Eigen::Upper>();
    for (int row = 0; row < Hcols; ++row) {
      for (int col = 0; col < Hcols; ++col) {
        if (fabs(R(row, col)) < 1e-10)
          R(row, col) = 0;
      }
    }
    *T_H = R.block(0, 0, Hcols, Hcols);
  }
}

int FilterHelper::pruneSquareMatrix(int rm_state_start, int rm_state_end,
                                     Eigen::MatrixXd* state_cov) {
  // Remove the corresponding rows and columns in the state
  // covariance matrix.
  int new_cov_dim = state_cov->rows() - (rm_state_end - rm_state_start);
  if (rm_state_end < state_cov->rows()) {
    state_cov->block(rm_state_start, 0, state_cov->rows() - rm_state_end,
                     state_cov->cols()) =
        state_cov->block(rm_state_end, 0, state_cov->rows() - rm_state_end,
                         state_cov->cols());

    state_cov->block(0, rm_state_start, state_cov->rows(),
                     state_cov->cols() - rm_state_end) =
        state_cov->block(0, rm_state_end, state_cov->rows(),
                         state_cov->cols() - rm_state_end);

    state_cov->conservativeResize(new_cov_dim, new_cov_dim);
  } else {
    state_cov->conservativeResize(new_cov_dim, new_cov_dim);
  }
  return new_cov_dim;
}

Eigen::MatrixXd FilterHelper::leftNullspaceWithRankCheck(
    const Eigen::MatrixXd& A, int columnRankHint) {
  int rank = A.cols();
  while (A.col(rank - 1).norm() < 1e-4) {
    rank--;
  }
  rank = columnRankHint < rank ? columnRankHint : rank;

  Eigen::HouseholderQR<Eigen::MatrixXd> qr(A.leftCols(rank));
  Eigen::MatrixXd nullQ = qr.householderQ();

  int rows = A.rows();
  nullQ = nullQ.block(0, rank, rows, rows - rank).eval();
  return nullQ;
}

bool FilterHelper::multiplyLeftNullspaceWithGivens(
    Eigen::MatrixXd* Hf, Eigen::MatrixXd* Hx,
    Eigen::Matrix<double, Eigen::Dynamic, 1>* residual, Eigen::MatrixXd* R,
    int columnRankHint) {
  int M = Hf->rows();
  int N = Hf->cols();
  if (Hf->col(N - 1).norm() < 1e-4) {
    N--;
  }
  N = columnRankHint < N ? columnRankHint : N;
  Eigen::JacobiRotation<double> Hf_GR;
  for (int n = 0; n < N; ++n) {
    for (int m = M - 1; m > n; m--) {
      // Givens matrix G
      Hf_GR.makeGivens((*Hf)(m - 1, n), (*Hf)(m, n));

      // Multiply G' to the corresponding lines (m-1,m) in each matrix

      // Hf
      // Note: we only apply G' to the nonzero cols [n:N-1], which is
      //       equivalent to applying G' to the entire row [0:N-1].
      (Hf->block(m - 1, n, 2, N - n)).applyOnTheLeft(0, 1, Hf_GR.adjoint());

      // G'*Hx
      (Hx->block(m - 1, 0, 2, Hx->cols()))
          .applyOnTheLeft(0, 1, Hf_GR.adjoint());

      // G'*r
      (residual->block(m - 1, 0, 2, 1)).applyOnTheLeft(0, 1, Hf_GR.adjoint());
    }
  }
  // remove the zero section
  int nDOF = M - N;
  *residual = residual->block(N, 0, nDOF, 1);
  *Hx = Hx->block(N, 0, nDOF, Hx->cols());
  // TODO(jhuai): apply Givens to R
  R->setZero(nDOF, nDOF);
  R->diagonal().setOnes();
  return true;
}

const double FilterHelper::chi2_95percentile[] = {
      0,  // for easy reference at degree 0
      3.841459,   5.991465,   7.814728,   9.487729,  11.070498,  12.591587,  14.067140,  15.507313,  16.918978,  18.307038,
     19.675138,  21.026070,  22.362032,  23.684791,  24.995790,  26.296228,  27.587112,  28.869299,  30.143527,  31.410433,
     32.670573,  33.924438,  35.172462,  36.415029,  37.652484,  38.885139,  40.113272,  41.337138,  42.556968,  43.772972,
     44.985343,  46.194260,  47.399884,  48.602367,  49.801850,  50.998460,  52.192320,  53.383541,  54.572228,  55.758479,
     56.942387,  58.124038,  59.303512,  60.480887,  61.656233,  62.829620,  64.001112,  65.170769,  66.338649,  67.504807,
     68.669294,  69.832160,  70.993453,  72.153216,  73.311493,  74.468324,  75.623748,  76.777803,  77.930524,  79.081944,
     80.232098,  81.381015,  82.528727,  83.675261,  84.820645,  85.964907,  87.108072,  88.250164,  89.391208,  90.531225,
     91.670239,  92.808270,  93.945340,  95.081467,  96.216671,  97.350970,  98.484383,  99.616927, 100.748619, 101.879474,
    103.009509, 104.138738, 105.267177, 106.394840, 107.521741, 108.647893, 109.773309, 110.898003, 112.021986, 113.145270,
    114.267868, 115.389790, 116.511047, 117.631651, 118.751612, 119.870939, 120.989644, 122.107735, 123.225221, 124.342113,
    125.458419, 126.574148, 127.689308, 128.803908, 129.917955, 131.031458, 132.144425, 133.256862, 134.368777, 135.480178,
    136.591071, 137.701464, 138.811363, 139.920774, 141.029704, 142.138160, 143.246147, 144.353672, 145.460740, 146.567358,
    147.673530, 148.779262, 149.884561, 150.989430, 152.093876, 153.197903, 154.301516, 155.404721, 156.507522, 157.609923,
    158.711930, 159.813547, 160.914778, 162.015628, 163.116101, 164.216201, 165.315932, 166.415299, 167.514305, 168.612954,
    169.711251, 170.809198, 171.906799, 173.004059, 174.100981, 175.197567, 176.293823, 177.389750, 178.485353, 179.580634,
    180.675597, 181.770246, 182.864582, 183.958610, 185.052332, 186.145751, 187.238870, 188.331692, 189.424220, 190.516457,
    191.608404, 192.700066, 193.791445, 194.882542, 195.973362, 197.063906, 198.154177, 199.244177, 200.333909, 201.423375,
    202.512577, 203.601519, 204.690201, 205.778627, 206.866798, 207.954717, 209.042386, 210.129807, 211.216982, 212.303913,
    213.390602, 214.477052, 215.563263, 216.649239, 217.734981, 218.820491, 219.905770, 220.990822, 222.075646, 223.160247,
    224.244624, 225.328780, 226.412716, 227.496435, 228.579938, 229.663226, 230.746302, 231.829167, 232.911822, 233.994269,
    235.076510, 236.158546, 237.240378, 238.322009, 239.403439, 240.484671, 241.565705, 242.646544, 243.727187, 244.807638,
    245.887897, 246.967965, 248.047844, 249.127536, 250.207041, 251.286361, 252.365498, 253.444451, 254.523224, 255.601816,
    256.680230, 257.758465, 258.836525, 259.914409, 260.992120, 262.069657, 263.147023, 264.224218, 265.301243, 266.378101,
    267.454791, 268.531314, 269.607673, 270.683868, 271.759900, 272.835769, 273.911478, 274.987027, 276.062417, 277.137650,
    278.212725, 279.287644, 280.362409, 281.437019, 282.511477, 283.585782, 284.659936, 285.733940, 286.807794, 287.881501,
    288.955059, 290.028471, 291.101737, 292.174858, 293.247835, 294.320669, 295.393360, 296.465910, 297.538319, 298.610588,
    299.682719, 300.754710, 301.826565, 302.898282, 303.969864, 305.041310, 306.112622, 307.183800, 308.254846, 309.325759,
    310.396541, 311.467192, 312.537713, 313.608105, 314.678368, 315.748503, 316.818512, 317.888393, 318.958149, 320.027780,
    321.097286, 322.166669, 323.235928, 324.305065, 325.374080, 326.442974, 327.511748, 328.580401, 329.648936, 330.717351,
    331.785649, 332.853829, 333.921892, 334.989839, 336.057670, 337.125386, 338.192988, 339.260476, 340.327850, 341.395112,
    342.462262, 343.529300, 344.596226, 345.663043, 346.729749, 347.796346, 348.862834, 349.929214, 350.995485, 352.061650,
    353.127708, 354.193659, 355.259504, 356.325245, 357.390880, 358.456412, 359.521839, 360.587163, 361.652385, 362.717504,
    363.782521, 364.847437, 365.912253, 366.976967, 368.041582, 369.106097, 370.170513, 371.234831, 372.299051, 373.363173,
    374.427197, 375.491125, 376.554957, 377.618692, 378.682332, 379.745878, 380.809328, 381.872684, 382.935947, 383.999116,
    385.062192, 386.125175, 387.188067, 388.250867, 389.313575, 390.376192, 391.438719, 392.501156, 393.563503, 394.625760,
    395.687929, 396.750009, 397.812000, 398.873904, 399.935720, 400.997450, 402.059092, 403.120648, 404.182118, 405.243502,
    406.304801, 407.366015, 408.427145, 409.488190, 410.549151, 411.610029, 412.670823, 413.731535, 414.792164, 415.852711,
    416.913176, 417.973559, 419.033862, 420.094083, 421.154224, 422.214284, 423.274265, 424.334166, 425.393988, 426.453731,
    427.513395, 428.572980, 429.632488, 430.691918, 431.751271, 432.810546, 433.869745, 434.928867, 435.987913, 437.046882,
    438.105777, 439.164596, 440.223339, 441.282008, 442.340603, 443.399123, 444.457570, 445.515942, 446.574242, 447.632468,
    448.690621, 449.748702, 450.806711, 451.864647, 452.922512, 453.980305, 455.038027, 456.095679, 457.153259, 458.210769,
    459.268209, 460.325579, 461.382879, 462.440110, 463.497272, 464.554365, 465.611389, 466.668344, 467.725232, 468.782052,
    469.838804, 470.895488, 471.952105, 473.008656, 474.065139, 475.121556, 476.177907, 477.234192, 478.290411, 479.346565,
    480.402653, 481.458676, 482.514634, 483.570528, 484.626357, 485.682122, 486.737823, 487.793460, 488.849033, 489.904544,
    490.959991, 492.015375, 493.070697, 494.125956, 495.181153, 496.236287, 497.291360, 498.346372, 499.401322, 500.456210,
    501.511038, 502.565805, 503.620511, 504.675157, 505.729742, 506.784268, 507.838733, 508.893140, 509.947486, 511.001774,
    512.056002, 513.110172, 514.164283, 515.218335, 516.272329, 517.326265, 518.380143, 519.433964, 520.487727, 521.541432,
    522.595081, 523.648672, 524.702207, 525.755685, 526.809107, 527.862472, 528.915781, 529.969035, 531.022232, 532.075374,
    533.128461, 534.181492, 535.234469, 536.287390, 537.340257, 538.393069, 539.445827, 540.498531, 541.551181, 542.603777,
    543.656319, 544.708807, 545.761243, 546.813625, 547.865954, 548.918230, 549.970453, 551.022624, 552.074743, 553.126809
};

bool FilterHelper::gatingTest(const Eigen::MatrixXd &H, const Eigen::VectorXd &r,
                              const Eigen::MatrixXd &R, const Eigen::MatrixXd &cov,
                              bool useMahalanobis) {
  if (useMahalanobis) {
    double gamma =
        r.transpose() * (H * cov * H.transpose() + R).ldlt().solve(r);
    if (gamma < chi2_95percentile[r.rows()]) {
      return true;
    } else {
      return false;
    }
  } else {
    return true;
  }
}

void FilterHelper::addCovForClonedNavState(Eigen::MatrixXd *covariance, size_t insertionIndex) {
  int oldCovDim = covariance->rows();
  const size_t numPointStates = oldCovDim - insertionIndex;

  size_t covDimAugmented = oldCovDim + 9;  //$\delta p,\delta \alpha,\delta v$
  Eigen::MatrixXd covarianceAugmented(covDimAugmented, covDimAugmented);
  covarianceAugmented.topLeftCorner(insertionIndex,
                                    insertionIndex) =
      covariance->topLeftCorner(insertionIndex,
                                insertionIndex);

  covarianceAugmented.block(0, insertionIndex,
                            insertionIndex, 9) =
      covariance->topLeftCorner(insertionIndex, 9);

  if (numPointStates > 0) {
    covarianceAugmented.topRightCorner(insertionIndex,
                                       numPointStates) =
        covariance->topRightCorner(insertionIndex, numPointStates);

    covarianceAugmented.bottomLeftCorner(numPointStates,
                                         insertionIndex) =
        covariance->bottomLeftCorner(numPointStates, insertionIndex);

    covarianceAugmented.block(insertionIndex + 9,
                              insertionIndex, numPointStates, 9) =
        covariance->bottomLeftCorner(numPointStates, 9);

    covarianceAugmented.bottomRightCorner(numPointStates, numPointStates) =
        covariance->bottomRightCorner(numPointStates, numPointStates);
  }

  covarianceAugmented.block(insertionIndex, 0, 9, covDimAugmented) =
      covarianceAugmented.topLeftCorner(9, covDimAugmented);

  *covariance = covarianceAugmented;
}

void FilterHelper::initializeClonedNavStateCov(StateMap *stateMap, Eigen::MatrixXd *covariance,
                                               size_t clonedStateStartIdx, size_t gravityIdx) {
  int oldCovSize = covariance->rows();
  int deltaRows = stateMap->size() * 9;
  int newCovSize = oldCovSize + deltaRows;
  int clonedStateIdx = clonedStateStartIdx;
  covariance->conservativeResize(newCovSize, newCovSize);
  covariance->block(oldCovSize, 0, deltaRows, newCovSize).setZero();
  covariance->block(0, oldCovSize, oldCovSize, deltaRows).setZero();

  int counter = 0;
  double poseCovDelta = 4e-4;
  double velCovDelta = 1e-4;
  for (StateMap::iterator iter = stateMap->begin(); iter != stateMap->end();
       ++iter) {
    covariance->block<6, 6>(clonedStateStartIdx, clonedStateStartIdx) =
        covariance->topLeftCorner<6, 6>();
    covariance->block<6, 6>(clonedStateStartIdx, clonedStateStartIdx) +=
        (poseCovDelta * counter * Eigen::Matrix<double, 6, 1>::Ones())
            .asDiagonal();
    covariance->block<3, 3>(clonedStateStartIdx + 6, clonedStateStartIdx + 6) =
        covariance->block<3, 3>(6, 6);
    covariance->block<3, 3>(clonedStateStartIdx + 6, clonedStateStartIdx + 6) +=
        (velCovDelta * counter * Eigen::Matrix<double, 3, 1>::Ones())
            .asDiagonal();

    iter->second.global.at(GlobalStates::T_WS).startIndexInCov =
        clonedStateStartIdx;
    iter->second.global.at(GlobalStates::v_WS).startIndexInCov =
        clonedStateStartIdx + 6;
    iter->second.global.at(GlobalStates::GravityDirection).startIndexInCov =
        gravityIdx;
    clonedStateStartIdx += 9;
    ++counter;
  }

  // initialize off diagonal blocks for the cloned nav states
//  for (size_t i = 0; i < stateMap->size(); ++i) {
//    for (size_t j = i + 1; j < stateMap->size(); ++j) {
//      size_t counter = j - i;
//      covariance->block(i * 9 + clonedStateIdx, j * 9 + clonedStateIdx, 6, 6) =
//          covariance->topLeftCorner<6, 6>();
//      covariance->block(i * 9 + clonedStateIdx, j * 9 + clonedStateIdx, 6, 6) +=
//          (0.5 * poseCovDelta * counter * Eigen::Matrix<double, 6, 1>::Ones()).asDiagonal();
//      covariance->block(i * 9 + clonedStateIdx + 6, j * 9 + clonedStateIdx + 6,
//                        3, 3) = covariance->block<3, 3>(6, 6);
//      covariance->block(i * 9 + clonedStateIdx + 6, j * 9 + clonedStateIdx + 6,
//                        3, 3) +=
//          (0.5 * velCovDelta * counter * Eigen::Matrix<double, 3, 1>::Ones()).asDiagonal();

//      covariance->block(j * 9 + clonedStateIdx, i * 9 + clonedStateIdx, 9, 9) =
//          covariance->block(i * 9 + clonedStateIdx, j * 9 + clonedStateIdx, 9,
//                            9).transpose();
//    }
//  }

  // initialize off diagonal blocks for the first nav state
  for (size_t i = 0; i < stateMap->size(); ++i) {
      covariance->block(0, i * 9 + clonedStateIdx, 9, 9) =
          covariance->block<9, 9>((stateMap->size() - 1) * 9 + clonedStateIdx, i * 9 + clonedStateIdx);
      covariance->block(i * 9 + clonedStateIdx, 0, 9, 9) =
          covariance->block(0, i * 9 + clonedStateIdx, 9, 9).transpose();
  }

  // refresh cov block for the first nav state
  covariance->block(0, 0, 9, 9) =
      covariance->block<9, 9>((stateMap->size() - 1) * 9 + clonedStateIdx, (stateMap->size() - 1) * 9 + clonedStateIdx);
}

void FilterHelper::saveCovariance(const Eigen::MatrixXd &cov, const std::string &outputdir) {
  std::string covfile = outputdir + "/cov.txt";
  std::ofstream fs(covfile);
  fs << "#cov rows " << cov.rows() << "\n";
  fs << cov.format(swift_vio::kSpaceInitFmt) << "\n";
  fs.close();
}

void FilterHelper::initializeNavStateCov(const InitialNavState &initialNavState,
                                         const okvis::ImuParameters &imuParams, Eigen::MatrixXd *covariance, size_t gWStartIndex) {
  covariance->topLeftCorner<3, 3>() = initialNavState.sigma_p_WS.cwiseAbs2().asDiagonal();
  covariance->block<3, 3>(3, 3) = initialNavState.sigma_q_WS.cwiseAbs2().asDiagonal();
  covariance->block<3, 3>(6, 6) = initialNavState.sigma_v_WS.cwiseAbs2().asDiagonal();
  if (imuParams.isGravityDirectionVariable())
    covariance->block<2, 2>(gWStartIndex, gWStartIndex) =
        imuParams.sigma_gravity_direction * imuParams.sigma_gravity_direction *
        Eigen::Matrix2d::Identity();
}

void FilterHelper::initializeImuStateCov(
    const std::vector<std::shared_ptr<okvis::ImuParameters>> &imuParametersVec,
    const ImuRig &imuRig,
    Eigen::MatrixXd *covariance, States *currentState, size_t imuParamStartIdx) {
  for (size_t i = 0; i < imuParametersVec.size(); ++i) {
    const double sigma_bg = imuParametersVec.at(i)->sigma_bg;
    const double sigma_ba = imuParametersVec.at(i)->sigma_ba;
    const double gyrBiasVariance = sigma_bg * sigma_bg,
                 accBiasVariance = sigma_ba * sigma_ba;

    covariance->block<3, 3>(imuParamStartIdx, imuParamStartIdx) = Eigen::Matrix3d::Identity() * gyrBiasVariance;
    covariance->block<3, 3>(imuParamStartIdx + 3, imuParamStartIdx + 3) =
        Eigen::Matrix3d::Identity() * accBiasVariance;
    SpecificSensorStatesContainer &imuInfo =
        currentState->sensors.at(SensorStates::Imu).at(i);
    imuInfo.at(ImuSensorStates::Bias).startIndexInCov = imuParamStartIdx;
    imuParamStartIdx = imuParamStartIdx + 6;

    int imuModelId = imuRig.getModelId(i);
    switch (imuModelId) {
    case Imu_BG_BA_MG_TS_MA::kModelId: {
      const double sigma_Mg_element = imuParametersVec.at(i)->sigma_Mg_element;
      const double sigma_Ts_element = imuParametersVec.at(i)->sigma_Ts_element;
      const double sigma_Ma_element = imuParametersVec.at(i)->sigma_Ma_element;
      covariance->block<9, 9>(imuParamStartIdx, imuParamStartIdx) = (Eigen::Matrix<double, 9, 1>::Ones() *
                                    std::pow(sigma_Mg_element, 2)).asDiagonal();
      imuInfo.at(ImuSensorStates::MG).startIndexInCov = imuParamStartIdx;
      imuParamStartIdx = imuParamStartIdx + 9;
      covariance->block<9, 9>(imuParamStartIdx, imuParamStartIdx) = (Eigen::Matrix<double, 9, 1>::Ones() *
                                    std::pow(sigma_Ts_element, 2)).asDiagonal();
      imuInfo.at(ImuSensorStates::TS).startIndexInCov = imuParamStartIdx;
      imuParamStartIdx = imuParamStartIdx + 9;
      covariance->block<6, 6>(imuParamStartIdx, imuParamStartIdx) = (Eigen::Matrix<double, 6, 1>::Ones() *
                                    std::pow(sigma_Ma_element, 2)).asDiagonal();

      imuInfo.at(ImuSensorStates::MA).startIndexInCov = imuParamStartIdx;
      imuParamStartIdx = imuParamStartIdx + 6;
    } break;
    case Imu_BG_BA::kModelId:
      imuInfo.at(ImuSensorStates::MG).startIndexInCov = imuParamStartIdx;
      imuInfo.at(ImuSensorStates::TS).startIndexInCov = imuParamStartIdx;
      imuInfo.at(ImuSensorStates::MA).startIndexInCov = imuParamStartIdx;
      break;
    default:
      LOG(WARNING) << "Not implemented IMU model " << imuModelId << ".";
      break;
    }
  }
}

void FilterHelper::initializeCameraStateCov(
    const okvis::CameraNoiseParametersVec &cameraNoiseParametersVec,
    const CameraRig &cameraRig, Eigen::MatrixXd *covariance,
    States *currentState, size_t camParamStartIdx) {
  int camParamIndex = camParamStartIdx;
  for (size_t camIdx = 0u; camIdx < cameraRig.numCameras(); ++camIdx) {
    int minExtrinsicDim = cameraRig.getMinimalExtrinsicDim(camIdx);
    int minProjectionIntrinsicDim = cameraRig.getMinimalProjectionIntrinsicDim(camIdx);
    int distortionDim = cameraRig.getDistortionDim(camIdx);

    Eigen::MatrixXd covExtrinsic;

    SpecificSensorStatesContainer &cameraInfos =
        currentState->sensors.at(SensorStates::Camera).at(camIdx);

    cameraInfos.at(CameraSensorStates::T_XCi)
        .startIndexInCov = camParamIndex;
    if (!cameraNoiseParametersVec.at(camIdx).isExtrinsicsFixed()) {
      covExtrinsic = ExtrinsicRepInitCov(
          cameraRig.getExtrinsicRepId(camIdx),
          cameraNoiseParametersVec.at(camIdx).sigma_absolute_translation,
          cameraNoiseParametersVec.at(camIdx).sigma_absolute_orientation);
      covariance->block(camParamIndex, camParamIndex, minExtrinsicDim,
                        minExtrinsicDim) = covExtrinsic;
      camParamIndex += minExtrinsicDim;
    }
    cameraInfos.at(CameraSensorStates::Intrinsics).startIndexInCov = camParamIndex;
    if (!cameraNoiseParametersVec.at(camIdx).isIntrinsicsFixed()) {
      Eigen::MatrixXd covProjIntrinsics = ProjectionIntrinsicRepGetInitCov(
          cameraRig.getProjectionIntrinsicRepId(camIdx),
          cameraNoiseParametersVec.at(camIdx).sigma_focal_length,
          cameraNoiseParametersVec.at(camIdx).sigma_principal_point);

      Eigen::MatrixXd covDistortion = Eigen::MatrixXd::Identity(distortionDim, distortionDim);
      for (int jack = 0; jack < distortionDim; ++jack)
        covDistortion(jack, jack) *= std::pow(
            cameraNoiseParametersVec.at(camIdx).sigma_distortion[jack], 2);

      covariance->block(camParamIndex, camParamIndex, minProjectionIntrinsicDim,
                        minProjectionIntrinsicDim) = covProjIntrinsics;
      camParamIndex += minProjectionIntrinsicDim;

      covariance->block(camParamIndex, camParamIndex, distortionDim,
                        distortionDim) = covDistortion;
      camParamIndex += distortionDim;
    }
    cameraInfos.at(CameraSensorStates::TD).startIndexInCov = camParamIndex;
    if (!cameraNoiseParametersVec.at(camIdx).isTimeDelayFixed()) {
      (*covariance)(camParamIndex, camParamIndex) =
          std::pow(cameraNoiseParametersVec.at(camIdx).sigma_td, 2);
      ++camParamIndex;
    }
    cameraInfos.at(CameraSensorStates::TR).startIndexInCov = camParamIndex;
    if (!cameraNoiseParametersVec.at(camIdx).isReadoutTimeFixed()) {
      (*covariance)(camParamIndex, camParamIndex) =
          std::pow(cameraNoiseParametersVec.at(camIdx).sigma_tr, 2);
      ++camParamIndex;
    }
  }
}

void FilterHelper::updateImuStates(const States &currentState,
                                   const Eigen::VectorXd &deltaX,
                                   const ImuRig &imuRig,
                                   std::shared_ptr<okvis::ceres::Map> mapPtr) {
  for (size_t i = 0; i < currentState.sensors.at(SensorStates::Imu).size(); ++i) {
    const SpecificSensorStatesContainer &imuInfo =
        currentState.sensors.at(SensorStates::Imu).at(i);
    uint64_t bid = imuInfo.at(ImuSensorStates::Bias).id;
    std::shared_ptr<okvis::ceres::BiasParameterBlock> bParamBlockPtr =
        std::static_pointer_cast<okvis::ceres::BiasParameterBlock>(
            mapPtr->parameterBlockPtr(bid));
    Eigen::Matrix<double, 6, 1> bias = bParamBlockPtr->estimate();
    bParamBlockPtr->setEstimate(
        bias +
        deltaX.segment<6>(imuInfo.at(ImuSensorStates::Bias).startIndexInCov));

    int imuModelId = imuRig.getModelId(i);
    switch (imuModelId) {
    case Imu_BG_BA::kModelId:
      continue;
    case Imu_BG_BA_MG_TS_MA::kModelId: {
      uint64_t MgId = imuInfo.at(ImuSensorStates::MG).id;
      std::shared_ptr<okvis::ceres::ShapeMatrixParamBlock> MgParamBlockPtr =
          std::static_pointer_cast<okvis::ceres::ShapeMatrixParamBlock>(
              mapPtr->parameterBlockPtr(MgId));
      Eigen::Matrix<double, 9, 1> sm = MgParamBlockPtr->estimate();
      MgParamBlockPtr->setEstimate(
          sm +
          deltaX.segment<9>(imuInfo.at(ImuSensorStates::MG).startIndexInCov));

      uint64_t TsId = imuInfo.at(ImuSensorStates::TS).id;
      std::shared_ptr<okvis::ceres::ShapeMatrixParamBlock> TsParamBlockPtr =
          std::static_pointer_cast<okvis::ceres::ShapeMatrixParamBlock>(
              mapPtr->parameterBlockPtr(TsId));
      sm = TsParamBlockPtr->estimate();
      TsParamBlockPtr->setEstimate(
          sm +
          deltaX.segment<9>(imuInfo.at(ImuSensorStates::TS).startIndexInCov));

      uint64_t MaId = imuInfo.at(ImuSensorStates::MA).id;
      std::shared_ptr<okvis::ceres::EuclideanParamBlockSized<6>>
          MaParamBlockPtr = std::static_pointer_cast<
              okvis::ceres::EuclideanParamBlockSized<6>>(
              mapPtr->parameterBlockPtr(MaId));
      MaParamBlockPtr->setEstimate(
          MaParamBlockPtr->estimate() +
          deltaX.segment<6>(imuInfo.at(ImuSensorStates::MA).startIndexInCov));
    } break;
    default:
      LOG(WARNING) << "UpdateState for IMU model " << imuModelId
                   << " not implemented!";
      break;
    }
  }
}

void FilterHelper::updateCameraSensorStates(const States& currentState,
                                            const Eigen::VectorXd& deltaX,
                                            const CameraRig &cameraRig,
                                            const okvis::CameraNoiseParametersVec &cameraNoiseParametersVec,
                                            std::shared_ptr<okvis::ceres::Map> mapPtr) {
  const int numCameras = cameraRig.numCameras();
  for (int camIdx = 0; camIdx < numCameras; ++camIdx) {
    // It has been shown that estimating the extrinsic parameter T_ClCr performs
    // better than estimating T_SCr. see eq(5) in A comparative analysis of
    // tightly-coupled monocular, binocular, and stereo VINS.
    // Therefore, we may save T_C0Ci in place of T_BCi.
    const SpecificSensorStatesContainer &camInfo = currentState.sensors.at(SensorStates::Camera)
        .at(camIdx);
    if (!cameraNoiseParametersVec.at(camIdx).isExtrinsicsFixed()) {
      uint64_t extrinsicId = camInfo.at(CameraSensorStates::T_XCi).id;
      std::shared_ptr<okvis::ceres::PoseParameterBlock> extrinsicParamBlockPtr =
          std::static_pointer_cast<okvis::ceres::PoseParameterBlock>(
              mapPtr->parameterBlockPtr(extrinsicId));

      okvis::kinematics::Transformation T_XC = extrinsicParamBlockPtr->estimate();
      int extrinsicRepId = cameraRig.getExtrinsicRepId(camIdx);
      ExtrinsicRepOplus(extrinsicRepId, deltaX.data() + camInfo.at(CameraSensorStates::T_XCi).startIndexInCov, &T_XC);
      extrinsicParamBlockPtr->setEstimate(T_XC);
    }

    if (!cameraNoiseParametersVec.at(camIdx).isIntrinsicsFixed()) {
      const int intrinsicDim =
          cameraRig.getIntrinsicDim(camIdx);
      uint64_t intrinsicId = camInfo.at(CameraSensorStates::Intrinsics).id;
      std::shared_ptr<okvis::ceres::EuclideanParamBlock> intrinsicParamBlockPtr =
          std::static_pointer_cast<okvis::ceres::EuclideanParamBlock>(
              mapPtr->parameterBlockPtr(intrinsicId));
      Eigen::VectorXd cameraIntrinsics =
          intrinsicParamBlockPtr->estimate() +
          deltaX.segment(camInfo.at(CameraSensorStates::Intrinsics).startIndexInCov, intrinsicDim);
      intrinsicParamBlockPtr->setEstimate(cameraIntrinsics);
    }

    if (!cameraNoiseParametersVec.at(camIdx).isTimeDelayFixed()) {
      uint64_t tdId = camInfo.at(CameraSensorStates::TD).id;
      std::shared_ptr<okvis::ceres::ParameterBlock> tdParamBlockPtr =
          mapPtr->parameterBlockPtr(tdId);
      tdParamBlockPtr->parameters()[0] +=
          deltaX(camInfo.at(CameraSensorStates::TD).startIndexInCov);
    }

    if (!cameraNoiseParametersVec.at(camIdx).isReadoutTimeFixed()) {
      uint64_t trId = camInfo.at(CameraSensorStates::TR).id;
      std::shared_ptr<okvis::ceres::ParameterBlock> trParamBlockPtr =
          mapPtr->parameterBlockPtr(trId);
      trParamBlockPtr->parameters()[0] +=
          deltaX(camInfo.at(CameraSensorStates::TR).startIndexInCov);
    }
  }
}

void FilterHelper::composeErrorStateMap(
    const StateMap &statesMap, const ImuRig &imuRig,
    const okvis::ImuParameters &imuParams, const CameraRig &cameraRig,
    const okvis::CameraNoiseParametersVec &cameraNoiseParametersVec,
    std::unordered_map<uint64_t, ErrorStateInfo> *errorStateMap) {
  errorStateMap->clear();

  const States &currentState = statesMap.rbegin()->second;
  int startIndexInCov = 9;
  // IMUs
  int i = 0;
  for (const auto &imuInfo : currentState.sensors.at(SensorStates::Imu)) {
    int imuModelId = imuRig.getModelId(i);
    int minDim = 6;
    errorStateMap->emplace(
        std::piecewise_construct,
        std::forward_as_tuple(imuInfo[ImuSensorStates::Bias].id),
        std::forward_as_tuple(imuInfo[ImuSensorStates::Bias].id,
                              startIndexInCov, minDim));
    startIndexInCov += minDim;
    switch (imuModelId) {
    case Imu_BG_BA_MG_TS_MA::kModelId: {
      minDim = 9;
      errorStateMap->emplace(
          std::piecewise_construct,
          std::forward_as_tuple(imuInfo[ImuSensorStates::MG].id),
          std::forward_as_tuple(imuInfo[ImuSensorStates::MG].id,
                                startIndexInCov, minDim));
      startIndexInCov += minDim;
      minDim = 9;
      errorStateMap->emplace(
          std::piecewise_construct,
          std::forward_as_tuple(imuInfo[ImuSensorStates::TS].id),
          std::forward_as_tuple(imuInfo[ImuSensorStates::TS].id,
                                startIndexInCov, minDim));
      startIndexInCov += minDim;
      minDim = 6;
      errorStateMap->emplace(
          std::piecewise_construct,
          std::forward_as_tuple(imuInfo[ImuSensorStates::MA].id),
          std::forward_as_tuple(imuInfo[ImuSensorStates::MA].id,
                                startIndexInCov, minDim));
      startIndexInCov += minDim;
    } break;
    case Imu_BG_BA::kModelId:
      break;
    default:
      LOG(ERROR) << "Unsupported IMU type " << imuModelId << ".";
      break;
    }
    ++i;
  }

  // gravity
  if (imuParams.isGravityDirectionVariable()) {
    int minDim = 2;
    errorStateMap->emplace(
        std::piecewise_construct,
        std::forward_as_tuple(
            currentState.global.at(GlobalStates::GravityDirection).id),
        std::forward_as_tuple(
            currentState.global.at(GlobalStates::GravityDirection).id,
            startIndexInCov, minDim));
    startIndexInCov += minDim;
  }

  // cameras
  for (size_t camIdx = 0u;
       camIdx < currentState.sensors.at(SensorStates::Camera).size();
       ++camIdx) {
    int minExtrinsicDim = cameraRig.getMinimalExtrinsicDim(camIdx);
    int intrinsicDim = cameraRig.getIntrinsicDim(camIdx);

    const SpecificSensorStatesContainer &cameraInfos =
        currentState.sensors.at(SensorStates::Camera).at(camIdx);
    const okvis::CameraNoiseParameters &noiseParams =
        cameraNoiseParametersVec.at(camIdx);
    if (!noiseParams.isExtrinsicsFixed()) {
      int minDim = minExtrinsicDim;
      errorStateMap->emplace(
          std::piecewise_construct,
          std::forward_as_tuple(cameraInfos.at(CameraSensorStates::T_XCi).id),
          std::forward_as_tuple(cameraInfos.at(CameraSensorStates::T_XCi).id,
                                startIndexInCov, minDim));
      startIndexInCov += minDim;
    }
    if (!noiseParams.isIntrinsicsFixed()) {
      int minDim = intrinsicDim;
      errorStateMap->emplace(
          std::piecewise_construct,
          std::forward_as_tuple(
              cameraInfos.at(CameraSensorStates::Intrinsics).id),
          std::forward_as_tuple(
              cameraInfos.at(CameraSensorStates::Intrinsics).id,
              startIndexInCov, minDim));
      startIndexInCov += minDim;
    }
    if (!noiseParams.isTimeDelayFixed()) {
      int minDim = 1;
      errorStateMap->emplace(
          std::piecewise_construct,
          std::forward_as_tuple(cameraInfos.at(CameraSensorStates::TD).id),
          std::forward_as_tuple(cameraInfos.at(CameraSensorStates::TD).id,
                                startIndexInCov, minDim));
      startIndexInCov += minDim;
    }

    if (!noiseParams.isReadoutTimeFixed()) {
      int minDim = 1;
      errorStateMap->emplace(
          std::piecewise_construct,
          std::forward_as_tuple(cameraInfos.at(CameraSensorStates::TR).id),
          std::forward_as_tuple(cameraInfos.at(CameraSensorStates::TR).id,
                                startIndexInCov, minDim));
      startIndexInCov += minDim;
    }
  }

  // nav states
  int j = 0;
  for (auto it = statesMap.begin(); it != statesMap.end(); ++it) {
    const auto &stateInfo = it->second.global;
    int minDim = 6;
    errorStateMap->emplace(
        std::piecewise_construct,
        std::forward_as_tuple(stateInfo.at(GlobalStates::T_WS).id),
        std::forward_as_tuple(stateInfo.at(GlobalStates::T_WS).id,
                              startIndexInCov, minDim));
    startIndexInCov += minDim;
    minDim = 3;
    errorStateMap->emplace(
        std::piecewise_construct,
        std::forward_as_tuple(stateInfo.at(GlobalStates::v_WS).id),
        std::forward_as_tuple(stateInfo.at(GlobalStates::v_WS).id,
                              startIndexInCov, minDim));
    startIndexInCov += minDim;
    ++j;
  }
}

void FilterHelper::setOldErrorStateIndices(
    const std::vector<uint64_t> &oldVarIdList,
    std::unordered_map<uint64_t, ErrorStateInfo> *errorStateMap,
    size_t totalDim) {
  size_t oldStartIndexInCov = 0;
  for (auto varId : oldVarIdList) {
    auto &errorState = errorStateMap->at(varId);
    errorState.oldStartIndexInCov = oldStartIndexInCov;
    oldStartIndexInCov += errorState.minimalDim;
  }
  CHECK_EQ(oldStartIndexInCov, totalDim);
}

void FilterHelper::assignCovarianceIndices(
    const std::unordered_map<uint64_t, ErrorStateInfo> &errorStateMap,
    StateMap *stateMap) {
  States &currentState = stateMap->rbegin()->second;
  // sensor states: IMUs
  size_t nextVarStartIndexInCov = 0;
  for (SpecificSensorStatesContainer &sensorInfos :
       currentState.sensors.at(SensorStates::Imu)) {
    for (StateInfo &sinfo : sensorInfos) {
      if (sinfo.exists) {
        auto it = errorStateMap.find(sinfo.id);
        if (it != errorStateMap.end()) {
          sinfo.startIndexInCov = it->second.startIndexInCov;
          nextVarStartIndexInCov =
              sinfo.startIndexInCov + it->second.minimalDim;
        } else {
          sinfo.startIndexInCov = nextVarStartIndexInCov;
        }
      } else {
        sinfo.startIndexInCov = nextVarStartIndexInCov;
      }
    }
  }
  // gravity
  size_t gravityIndexInCov = 0;
  uint64_t gravityBlockId =
      currentState.global.at(GlobalStates::GravityDirection).id;
  auto it = errorStateMap.find(gravityBlockId);
  if (it != errorStateMap.end()) {
    gravityIndexInCov = it->second.startIndexInCov;
    nextVarStartIndexInCov = gravityIndexInCov + it->second.minimalDim;
  } else {
    gravityIndexInCov = nextVarStartIndexInCov;
  }
  // cameras
  for (SpecificSensorStatesContainer &sensorInfos :
       currentState.sensors.at(SensorStates::Camera)) {
    for (StateInfo &sinfo : sensorInfos) {
      if (sinfo.exists) {
        auto it = errorStateMap.find(sinfo.id);
        if (it != errorStateMap.end()) {
          sinfo.startIndexInCov = it->second.startIndexInCov;
          nextVarStartIndexInCov =
              sinfo.startIndexInCov + it->second.minimalDim;
        } else {
          sinfo.startIndexInCov = nextVarStartIndexInCov;
        }
      } else {
        sinfo.startIndexInCov = nextVarStartIndexInCov;
      }
    }
  }
  // nav states
  for (StateMap::iterator it = stateMap->begin(); it != stateMap->end(); ++it) {
    GlobalStatesContainer &globals = it->second.global;
    globals.at(GlobalStates::T_WS).startIndexInCov =
        errorStateMap.at(globals.at(GlobalStates::T_WS).id).startIndexInCov;
    globals.at(GlobalStates::v_WS).startIndexInCov =
        errorStateMap.at(globals.at(GlobalStates::v_WS).id).startIndexInCov;
    globals.at(GlobalStates::GravityDirection).startIndexInCov =
        gravityIndexInCov;
  }
}

}  // namespace swift_vio
