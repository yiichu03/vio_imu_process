
def honorv10_1280_options():
    algo_option_templates = {
        'KSWF': {"algo_code": "SlidingWindowFilter",
                 "extra_gflags": "--publish_via_ros=false",
                 "displayImages": "false",
                 "monocular_input": 1,
                 "numImuFrames": 5,
                 "minTrackLengthForSlam": 5,
                 "sigma_absolute_translation": 0.02,
                 "sigma_absolute_orientation": 0.01,
                 "sigma_g_c": 12.0e-4 * 2,
                 "sigma_a_c": 8.0e-3 * 2,
                 "sigma_gw_c": 2.0e-5,
                 "sigma_aw_c": 5.5e-5,
                 "loop_closure_method": 0},
    }

    config_name_to_diffs = {
        ('KSWF', 'KSWF'): {},
        ('SL-KSWF', "KSWF"): {"algo_code": "MSCKF", },
        ('OKVIS', 'KSWF'): {
            "algo_code": "OKVIS",
            "numImuFrames": 3,
            "sigma_c_relative_translation": 1.0e-6,
            "sigma_c_relative_orientation": 1.0e-6,
            "extrinsic_rep_main_camera": "p_BC_q_BC",
            "extrinsic_rep_other_camera": "p_BC_q_BC",
            "timeLimit": -1,
        }
    }
    return algo_option_templates, config_name_to_diffs


def zed2_openvins_options():
    algo_option_templates = {
        'OpenVINS': {"algo_code": "OpenVINS",
                     "launch_file": "pgeneva_serial_zed2_640.launch",
                     "gyroscope_noise_density": 2.8e-3,
                     "gyroscope_random_walk": 2.0e-5,
                     "accelerometer_noise_density": 1.6e-2,
                     "accelerometer_random_walk": 5.5e-5,
                     "calib_cam_extrinsics": "true",
                     "calib_cam_intrinsics": "false",
                     "calib_cam_timeoffset": "true",
                     },
    }

    config_name_to_diffs = {
        ('SLAM-Mono', 'OpenVINS'): {
            "max_cameras": 1,
            "use_stereo": 'false',
            "max_slam": "75",
            "max_slam_in_update": "25",
        },
    }
    return algo_option_templates, config_name_to_diffs


def kinect_azure_openvins_options():
    algo_option_templates = {
        'OpenVINS': {"algo_code": "OpenVINS",
                     "launch_file": "pgeneva_serial_kinect_azure.launch",
                     "gyroscope_noise_density": 1.2e-3,
                     "gyroscope_random_walk": 2.0e-05,
                     "accelerometer_noise_density": 8.0e-3,
                     "accelerometer_random_walk": 5.5e-5,
                     "calib_cam_extrinsics": "true",
                     "calib_cam_intrinsics": "false",
                     "calib_cam_timeoffset": "true",
                     },
    }

    config_name_to_diffs = {
        ('SLAM-Mono', 'OpenVINS'): {
            "max_cameras": 1,
            "use_stereo": 'false',
            "max_slam": "75",
            "max_slam_in_update": "25",
        },
    }
    return algo_option_templates, config_name_to_diffs
