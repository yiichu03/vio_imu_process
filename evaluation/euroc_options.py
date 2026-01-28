#!/usr/bin/env python3
# -*- coding: utf-8 -*-


def euroc_timing_swiftvio_options():
    """
    Run over the entire EUROC benchmark to time KSWF modules.
    stereo: KSWF full calib
    KSWF no sensor intrinsic calib
    Structureless KSWF no sensor intrinsic calib
    mono: KSWF full calib
    KSWF no sensor intrinsic calib
    structureless KSWF no sensor intrinsic calib
    :return:
    """
    algo_option_templates = {
        'KSWF': {"algo_code": "SlidingWindowFilter",
                 "extra_gflags": "--publish_via_ros=false",
                 "displayImages": "false",
                 "numImuFrames": 5,
                 "model_name": "BG_BA_MG_TS_MA",
                 "numThreads": 1,
                 "monocular_input": 0,
                 "loop_closure_method": 0},
    }

    config_name_to_diffs = {
        ('KSWF', 'KSWF'): {},
        ('KSWF-calib-extr', "KSWF"): {
            "sigma_tr": 0.0,
            "sigma_td": 0.0,
            "sigma_focal_length": 0.0,
            "sigma_principal_point": 0.0,
            "sigma_distortion": [0.0, 0.0, 0.0, 0.0, 0.0],
            "sigma_Mg_element": 0.0,
            "sigma_Ts_element": 0.0,
            "sigma_Ma_element": 0.0,
            "model_name": "BG_BA",
        },
        ('SL-KSWF-calib-extr', 'KSWF'): {
            "minTrackLengthForSlam": 1000,
            "sigma_tr": 0.0,
            "sigma_td": 0.0,
            "sigma_focal_length": 0.0,
            "sigma_principal_point": 0.0,
            "sigma_distortion": [0.0, 0.0, 0.0, 0.0, 0.0],
            "sigma_Mg_element": 0.0,
            "sigma_Ts_element": 0.0,
            "sigma_Ma_element": 0.0,
            "model_name": "BG_BA",
        },
        ('moKSWF', 'KSWF'): {
            "monocular_input": 1,
        },
        ('moKSWF-calib-extr', "KSWF"): {
            "monocular_input": 1,
            "sigma_tr": 0.0,
            "sigma_td": 0.0,
            "sigma_focal_length": 0.0,
            "sigma_principal_point": 0.0,
            "sigma_distortion": [0.0, 0.0, 0.0, 0.0, 0.0],
            "sigma_Mg_element": 0.0,
            "sigma_Ts_element": 0.0,
            "sigma_Ma_element": 0.0,
            "model_name": "BG_BA",
        },
        ('moSL-KSWF-calib-extr', 'KSWF'): {
            "monocular_input": 1,
            "minTrackLengthForSlam": 1000,
            "sigma_tr": 0.0,
            "sigma_td": 0.0,
            "sigma_focal_length": 0.0,
            "sigma_principal_point": 0.0,
            "sigma_distortion": [0.0, 0.0, 0.0, 0.0, 0.0],
            "sigma_Mg_element": 0.0,
            "sigma_Ts_element": 0.0,
            "sigma_Ma_element": 0.0,
            "model_name": "BG_BA",
        },
    }
    return algo_option_templates, config_name_to_diffs
