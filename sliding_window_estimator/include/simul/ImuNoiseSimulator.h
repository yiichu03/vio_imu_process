#ifndef IMU_SIMULATOR_H_
#define IMU_SIMULATOR_H_

#include "okvis/ImuMeasurements.hpp"
#include "okvis/Parameters.hpp"
#include <simul/SimParameters.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <fstream>
#include <iostream>
#include <vector>

namespace simul {
void initImuNoiseParams(const std::string &imuModel, double sim_sigma_g_c,
                        double sim_sigma_a_c, double sim_sigma_gw_c,
                        double sim_sigma_aw_c, okvis::ImuParameters *imuNoiseParams);

/**
 * @brief addNoiseToImuReadings
 * @param imuParameters
 * @param imuMeasurements as input original perfect imu measurement,
 *     as output imu measurements with added bias and noise
 * @param trueBiases output added biases
 * @param inertialStream
 */
void addNoiseToImuReadings(const okvis::ImuParameters& imuParameters,
                           okvis::ImuMeasurementDeque* imuMeasurements,
                           okvis::ImuMeasurementDeque* trueBiases,
                           std::ofstream* inertialStream);



} // namespace simul
#endif
