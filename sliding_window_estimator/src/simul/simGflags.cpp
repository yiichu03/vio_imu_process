#include <simul/simGflags.hpp>

DEFINE_string(sim_data_path, "",
              "Case 1 Directory of VIO data in maplab csv format generated from "
              "real data; Case 2 trajectory (T_WB) text file in TUM format!");

DEFINE_string(sim_algorithm, "SlidingWindowFilter", "Run the sliding window filter");

DEFINE_double(sim_sigma_speed, 0.05,
    "add noise to the initial value of velocity which is used to initialize an estimator.");

DEFINE_int32(sim_num_runs, 2, "How many times to run one simulation?");

DEFINE_double(sim_sigma_g_c, 1.2e-3, "simulated gyro noise density");

DEFINE_double(sim_sigma_a_c, 8e-3, "simulated accelerometer noise density");

DEFINE_double(sim_sigma_gw_c, 2e-5, "simulated gyro bias noise density");

DEFINE_double(sim_sigma_aw_c, 5.5e-5, "simulated accelerometer bias noise density");

DEFINE_string(sim_trajectory_label, "WavyCircle",
              "Ball has the most exciting motion, wavycircle is general");

DEFINE_int32(sim_landmark_model, 1,
             "Landmark model 0 for global homogeneous point, 1 for anchored "
             "inverse depth point, 2 for parallax angle parameterization");

DEFINE_double(sim_max_position_Rmse, 100, "If the final position RMSE is greater, then the run will be considered failed.");

DEFINE_string(
    sim_distortion_type, "RadialTangentialDistortion",
    "Distortion type for the simulated camera model when external sim data are "
    "not used. Available options: RadialTangentialDistortion, EquidistantDistortion, FovDistortion, EUCM");

DEFINE_string(
    sim_landmark_csv, "",
    "csv of landmarks which are used for simulation.");

DEFINE_string(
    sim_trajectory_csv, "",
    "csv of sampled poses for simulation.");
