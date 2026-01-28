
/**
 * @file SimParametersReader.hpp
 * @brief Header file for the SimParametersReader class.
 * @author
 */

#ifndef INCLUDE_SWIFT_VIO_SIMPARAMETERSREADER_HPP_
#define INCLUDE_SWIFT_VIO_SIMPARAMETERSREADER_HPP_

#include <string>

#include <simul/SimParameters.h>
#include <okvis/VioParametersReader.hpp>


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <ros/ros.h>
#pragma GCC diagnostic pop
#include <ros/callback_queue.h>

namespace simul {
/**
 * @brief This class extends the VioParametersReader class so as to load simulation parameters.
 */
class SimParametersReader : public okvis::VioParametersReader {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief The default constructor.
  SimParametersReader();

  /**
   * @brief The constructor. This calls readConfigFile().
   * @param filename Configuration filename.
   */
  SimParametersReader(const std::string& filename);

  void readSimParameters(const std::string &filename);

  bool getSimParameters(simul::SimParameters *simParameters) const {
    if (readConfigFile_)
      *simParameters = simParameters_;
    return readConfigFile_;
  }

  bool getVioParameters(okvis::VioParameters *vioParameters) const {
    return okvis::VioParametersReader::getParameters(*vioParameters);
  }

private:
  simul::SimParameters simParameters_;
};
}  // namespace simul

#endif /* INCLUDE_SWIFT_VIO_SIMPARAMETERSREADER_HPP_ */
