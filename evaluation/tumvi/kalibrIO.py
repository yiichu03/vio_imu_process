import os

import itertools
import numpy as np
import mathUtils


def loadCameraImuParameters(kalibr_output_dir):
    # get immediate subfolders
    # https://stackoverflow.com/questions/800197/how-to-get-all-of-the-immediate-subdirectories-in-python
    subfolders = [f.path for f in os.scandir(kalibr_output_dir) if f.is_dir()]

    kalib_estimates = list()

    from ruamel.yaml import YAML
    yaml = YAML()
    yaml.version = (1, 2)
    yaml.default_flow_style = None
    yaml.indent(mapping=4, sequence=6, offset=4)

    for path in subfolders:
        imulist = [f.path for f in os.scandir(path) if f.is_file() and 'imu-' in f.path]
        camimulist = [f.path for f in os.scandir(path) if f.is_file() and 'camchain-imucam-' in f.path]

        print('imulist {}'.format(imulist))
        print('camlist {}'.format(camimulist))
        if not imulist:
            continue
        camimu_calib = dict()
        with open(camimulist[0], 'r') as camimu_config:
            camimu_calib = yaml.load(camimu_config)

        with open(imulist[0], 'r') as imu_config:
            imu_calib = yaml.load(imu_config)
            camimu_calib.update(imu_calib)
            print('mix calib {}'.format(camimu_calib))
        kalib_estimates.append(camimu_calib)
    return kalib_estimates


def seq2array(seq):
    return np.array(list(itertools.zip_longest(*seq, fillvalue=0))).T


def compute_se3_mean_and_std(kalibr_estimates, cam):
    num = len(kalibr_estimates)
    dim = 6 # position and orientation
    refTseq = kalibr_estimates[0][cam]["T_cam_imu"]
    refT = seq2array(refTseq)
    assert(refT.shape[0] == 4 and refT.shape[1] == 4)

    R_CB0 = refT[:3, :3]  # R_CB
    p_CB0 = refT[:3, 3]  # P_CB
    R_BC0 = R_CB0.T
    p_BC0 = np.matmul(R_BC0, -p_CB0)

    val = np.zeros((num, dim))
    for i, estimates in enumerate(kalibr_estimates):
        if i == 0:
            val[0, :] = np.zeros(6)
        else:
            Ti = seq2array(estimates[cam]["T_cam_imu"])
            Ri = Ti[:3, :3] # R_CB
            pi = Ti[:3, 3] # P_CB
            R_BC = Ri.T
            p_BC = np.matmul(R_BC, -pi)
            val[i, :3] = p_BC - p_BC0
            val[i, 3:] = mathUtils.unskew(np.matmul(R_BC, R_CB0) - np.eye(3))
    mean_error = np.average(val, axis=0)
    std = np.std(val, axis=0)
    meanp = mean_error[:3] + p_BC0
    meanR = np.matmul(mathUtils.axisangle2dcm(mean_error[3:]), R_BC0)
    meanq = mathUtils.dcm2quat(meanR)
    return meanp, meanq, std


def compute_euclidean_mean_and_std(kalibr_estimates, cam, param):
    num = len(kalibr_estimates)
    dim = len(kalibr_estimates[0][cam][param])
    val = np.zeros((num, dim))
    for i, estimates in enumerate(kalibr_estimates):
        val[i, :] = estimates[cam][param]
    print('euc array ', val)
    mean = np.average(val, axis=0)
    std = np.std(val, axis=0)
    return mean, std


def compute_1d_mean_and_std(kalibr_estimates, cam, param):
    num = len(kalibr_estimates)
    val = np.zeros(num)
    for i, estimates in enumerate(kalibr_estimates):
        val[i] = estimates[cam][param]
    mean = np.average(val)
    std = np.std(val)
    return mean, std


def convertKalibrToSwiftVioParamsMgTsMa(kalibr_estimates):
    # compute mean and std for kalibr_estimates
    kswf_params = dict()
    kswf_params['bg'] = np.zeros(3)
    kswf_params['ba'] = np.zeros(3)

    kswf_params['std_bg'] = np.zeros(3)
    kswf_params['std_ba'] = np.zeros(3)

    numResults = len(kalibr_estimates)
    # Mg
    allvals = np.zeros((numResults, 9))
    for i, estimates in enumerate(kalibr_estimates):
        Mgp = np.matmul(seq2array(estimates['imu0']['gyroscopes']['M']),
                        seq2array(estimates['imu0']['gyroscopes']['C_gyro_i']))
        invMgp = np.linalg.inv(Mgp)
        allvals[i, :] = invMgp.reshape(9)
    kswf_params['Mg'] = np.average(allvals, axis=0)
    kswf_params['std_Mg'] = np.std(allvals, axis=0)
    # Ts
    allvals = np.zeros((numResults, 9))
    for i, estimates in enumerate(kalibr_estimates):
        Ts = np.matmul(seq2array(estimates['imu0']['gyroscopes']['A']),
                       seq2array(estimates['imu0']['gyroscopes']['C_gyro_i']))
        allvals[i, :] = Ts.reshape(9)
    kswf_params['Ts'] = np.average(allvals, axis=0)
    kswf_params['std_Ts'] = np.std(allvals, axis=0)
    # Ma, inverse of a lower triangular matrix is also lower triangular.
    allvals = np.zeros((numResults, 6))
    for i, estimates in enumerate(kalibr_estimates):
        Map = seq2array(estimates['imu0']['accelerometers']['M'])
        invMap = np.linalg.inv(Map)
        allvals[i, :] = np.array([invMap[0, 0], invMap[1, 0], invMap[1, 1],
                                  invMap[2, 0], invMap[2, 1], invMap[2, 2]])
    kswf_params['Ma'] = np.average(allvals, axis=0)
    kswf_params['std_Ma'] = np.std(allvals, axis=0)

    # T_BC0
    meanp, meanq, std = compute_se3_mean_and_std(kalibr_estimates, 'cam0')
    kswf_params['p_BC0'] = meanp
    kswf_params['q_BC0'] = meanq
    kswf_params['std_p_BC0'] = std[:3]
    kswf_params['std_q_BC0'] = std[3:]

    meanp, meanq, std = compute_se3_mean_and_std(kalibr_estimates, 'cam1')
    kswf_params['p_BC1'] = meanp
    kswf_params['q_BC1'] = meanq
    kswf_params['std_p_BC1'] = std[:3]
    kswf_params['std_q_BC1'] = std[3:]

    # fxy cxy
    mean, std = compute_euclidean_mean_and_std(kalibr_estimates, 'cam0', 'intrinsics')
    kswf_params['fc0'] = mean * 0.5  # from 1024 x 1024 to 512 x 512
    kswf_params['std_fc0'] = std * 0.5

    mean, std = compute_euclidean_mean_and_std(kalibr_estimates, 'cam0', 'distortion_coeffs')
    kswf_params['distort0'] = mean
    kswf_params['std_distort0'] = std

    mean, std = compute_1d_mean_and_std(kalibr_estimates, 'cam0', 'timeshift_cam_imu')
    kswf_params['tdtr0'] = [0, 0]
    kswf_params['std_tdtr0'] = [0, 0]

    mean, std = compute_euclidean_mean_and_std(kalibr_estimates, 'cam1', 'intrinsics')
    kswf_params['fc1'] = mean * 0.5
    kswf_params['std_fc1'] = std * 0.5
    print('fc2 std {}'.format(kswf_params['std_fc1']))

    mean, std = compute_euclidean_mean_and_std(kalibr_estimates, 'cam1', 'distortion_coeffs')
    kswf_params['distort1'] = mean
    kswf_params['std_distort1'] = std

    mean, std = compute_1d_mean_and_std(kalibr_estimates, 'cam1', 'timeshift_cam_imu')
    kswf_params['tdtr1'] = [0, 0]
    kswf_params['std_tdtr1'] = [0, 0]
    return kswf_params


# if __name__ == '__main__':
#     resdir = "/media/jhuai/OldWin8OS/jhuai/keyframe_based_filter/swift_vio_results/tumvi-calib-kalibr"
#     kalibr_estimates = loadCameraImuParameters(resdir)
#     kalibr_mean_and_std = convertKalibrToSwiftVioParamsMgTsMa(kalibr_estimates)
#     print('kalibr mean and std\n{}'.format(kalibr_mean_and_std))
