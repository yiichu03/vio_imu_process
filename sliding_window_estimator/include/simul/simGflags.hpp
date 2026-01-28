/**
 * @file simGflags.hpp
 * Gflags used in simulation.
 */

#ifndef SIMUL_GFLAGS_HPP
#define SIMUL_GFLAGS_HPP

#include <gflags/gflags.h>

DECLARE_string(sim_data_path);

DECLARE_string(sim_algorithm);

DECLARE_double(sim_sigma_speed);

DECLARE_int32(sim_num_runs);

DECLARE_double(sim_sigma_g_c);

DECLARE_double(sim_sigma_a_c);

DECLARE_double(sim_sigma_gw_c);

DECLARE_double(sim_sigma_aw_c);

DECLARE_string(sim_trajectory_label);

DECLARE_int32(sim_landmark_model);

DECLARE_string(sim_distortion_type);

DECLARE_string(sim_landmark_csv);

DECLARE_string(sim_trajectory_csv);

DECLARE_double(sim_max_position_Rmse);

#endif // SIMUL_GFLAGS_HPP
