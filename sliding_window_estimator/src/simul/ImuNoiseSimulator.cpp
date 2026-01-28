#include "simul/ImuNoiseSimulator.h"

#include <glog/logging.h>

#include "vio/Sample.h"


namespace simul {

void initImuNoiseParams(const std::string &imuModel, double sim_sigma_g_c,
                        double sim_sigma_a_c, double sim_sigma_gw_c,
                        double sim_sigma_aw_c, okvis::ImuParameters *imuNoiseParams) {
  imuNoiseParams->model_name = imuModel;
  imuNoiseParams->sigma_ba = 5e-3;
  imuNoiseParams->sigma_ba = 2e-2;
  imuNoiseParams->sigma_Mg_element = 5e-3;
  imuNoiseParams->sigma_Ts_element = 1e-3;
  imuNoiseParams->sigma_Ma_element = 5e-3;

  imuNoiseParams->sigma_g_c = sim_sigma_g_c;
  imuNoiseParams->sigma_gw_c = sim_sigma_gw_c;
  imuNoiseParams->sigma_a_c = sim_sigma_a_c;
  imuNoiseParams->sigma_aw_c = sim_sigma_aw_c;
}

void
addNoiseToImuReadings(const okvis::ImuParameters& imuParameters,
                           okvis::ImuMeasurementDeque* imuMeasurements,
                           okvis::ImuMeasurementDeque* trueBiases,
                           std::ofstream* inertialStream) {
  double sqrtRate = std::sqrt(imuParameters.rate);
  double sqrtDeltaT = 1 / sqrtRate;
  *trueBiases = (*imuMeasurements);
  Eigen::Vector3d bgk = imuParameters.initialGyroBias();
  Eigen::Vector3d bak = imuParameters.initialAccelBias();

  for (size_t i = 0; i < imuMeasurements->size(); ++i) {
    if (inertialStream) {
      Eigen::Vector3d porterGyro = imuMeasurements->at(i).measurement.gyroscopes;
      Eigen::Vector3d porterAcc = imuMeasurements->at(i).measurement.accelerometers;
      (*inertialStream) << imuMeasurements->at(i).timeStamp << " " << porterGyro[0]
                        << " " << porterGyro[1] << " " << porterGyro[2] << " "
                        << porterAcc[0] << " " << porterAcc[1] << " "
                        << porterAcc[2];
      (*inertialStream) << " " << bgk[0] << " " << bgk[1] << " " << bgk[2]
                        << " " << bak[0] << " " << bak[1] << " " << bak[2];
    }

    trueBiases->at(i).measurement.gyroscopes = bgk;
    trueBiases->at(i).measurement.accelerometers = bak;

    // eq 50, Oliver Woodman, An introduction to inertial navigation
    imuMeasurements->at(i).measurement.gyroscopes +=
        (bgk +
         vio::Sample::gaussian(imuParameters.sigma_g_c * sqrtRate,
                               3));
    imuMeasurements->at(i).measurement.accelerometers +=
        (bak +
         vio::Sample::gaussian(imuParameters.sigma_a_c * sqrtRate,
                               3));
    // eq 51, Oliver Woodman, An introduction to inertial navigation,
    // we do not divide sqrtDeltaT by sqrtT because sigma_gw_c is bias white noise density
    // for bias random walk (BRW) whereas eq 51 uses bias instability (BS) having the
    // same unit as the IMU measurements. also see eq 9 therein.
    bgk += vio::Sample::gaussian(
        imuParameters.sigma_gw_c * sqrtDeltaT, 3);
    bak += vio::Sample::gaussian(
        imuParameters.sigma_aw_c * sqrtDeltaT, 3);
    if (inertialStream) {
      Eigen::Vector3d porterGyro = imuMeasurements->at(i).measurement.gyroscopes;
      Eigen::Vector3d porterAcc = imuMeasurements->at(i).measurement.accelerometers;
      (*inertialStream) << " " << porterGyro[0] << " " << porterGyro[1] << " "
                        << porterGyro[2] << " " << porterAcc[0] << " "
                        << porterAcc[1] << " " << porterAcc[2] << std::endl;
    }
  }
}
}  // namespace simul
