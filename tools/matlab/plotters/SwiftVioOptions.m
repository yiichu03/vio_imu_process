function options = SwiftVioOptions(isfilter, numCameras, plotCamIdx, imumodel, cameraIntrinsicDims)
% set variable and standard deviation indices in the swift_vio output.
% isfilter: is the output from a filter or a smoother?
% numCameras: how many cameras are in the setup?
% plotCamIdx: to show which camera's parameters and their stds (1-based index).
% We only support drawing uncertainties for one camera.
% imumodel 0 for BG_BA, 1 for BG_BA_MG_TS_MA.

if nargin < 5
    cameraIntrinsicDims = [4, 4, 1, 1];
end

if nargin < 4
    imumodel = 0;
end
if nargin < 3
    plotCamIdx = 1;
end
if nargin < 2
    numCameras = 1;
end

projectionIntrinsicDim = cameraIntrinsicDims(1);
distortionIntrinsicDim = cameraIntrinsicDims(2);
tdDim = cameraIntrinsicDims(3);
trDim = cameraIntrinsicDims(4);

cameraParamDim = 3 + 4 + projectionIntrinsicDim + distortionIntrinsicDim + tdDim + trDim;
cameraParamMinDim = cameraParamDim - 1;

if imumodel == 1
    variableDimList = [3, 4, 3, 3, 3, 9, 9, 6];
else
    variableDimList = [3, 4, 3, 3, 3];
end
for i = 1:numCameras
    variableDimList = [variableDimList(:)', 3, 4, projectionIntrinsicDim, distortionIntrinsicDim, tdDim + trDim];
end

options.avg_since_start = 10;
options.avg_trim_end = 10;
options.trueCamProjectionIntrinsics = [0, 0, 0, 0];
options.trueCamDistortionIntrinsics = zeros(1, distortionIntrinsicDim);
% TUMVI raw sim
options.truep_camera = [0, 0, 0.05];
options.trueq_XC = [0, 0, -0.707107, 0.707107]; % wxyz
% curve sim
options.truep_camera = [0, 0, 0.0];
options.trueq_XC = [0.5, -0.5, 0.5, -0.5]; % wxyz for Forward camera orientation.

options.truep_camera = [0, 0, 0.0];
options.trueq_XC = [1.0, 0.0, 0.0, 0.0];
options.trueTimeOffset = 0.0;
options.trueReadoutTime = 0.0;

options.xlim = [];
options.camIntrinsicXlim = [];

% set indices for nav states and IMU biases and their std devs.
padding = 2;
options.r = padding + (1:3);
options.q = padding + (4:7);
options.v = padding + (8:10);
options.b_g = padding + (11:13);
options.b_a = padding + (14:16);

options.std_start_index = padding + sum(variableDimList);
options.r_std = options.std_start_index + (1:3);
options.q_std = options.std_start_index + (4:6);
options.v_std = options.std_start_index + (7:9);
options.b_g_std = options.std_start_index + (10:12);
options.b_a_std = options.std_start_index + (13:15);

% set indices for parameters.
options.T_g = [];
options.T_g_diag = [];
options.T_s = [];
options.T_s_diag = [];
options.T_a = [];
options.T_a_diag = [];

options.p_camera = [];
options.q_XC = [];
options.fxy_cxy = [];
options.k1_k2 = [];
options.p1_p2 = [];
options.td = [];
options.tr = [];

lastImuParamIndex = options.b_a(end);
if imumodel == 1
    options.T_g = 19:27;
    options.T_g_diag = [19, 23, 27];
    options.T_s = 28:36;
    options.T_s_diag = [28, 32, 36];
    options.T_a = 37:42;
    options.T_a_diag = [37, 39, 42];
    lastImuParamIndex = 42;
end

lastCameraParamIndex = lastImuParamIndex + cameraParamDim * (plotCamIdx - 1);
options.p_camera = lastCameraParamIndex + (1:3);
options.q_XC = options.p_camera(end) + (1:4);
lastCameraParamIndex = lastCameraParamIndex + 7;

if projectionIntrinsicDim
    assert(projectionIntrinsicDim == 4, 'only support 4 dim projection intrinsics!');
    options.fxy_cxy = lastCameraParamIndex + (1:projectionIntrinsicDim);
    lastCameraParamIndex = lastCameraParamIndex + projectionIntrinsicDim;
end

if distortionIntrinsicDim
    assert(distortionIntrinsicDim == 4, 'only support 4 dim distortion!');
    options.k1_k2 = lastCameraParamIndex + (1:2);
    options.p1_p2 = lastCameraParamIndex + (3:4);
    lastCameraParamIndex = lastCameraParamIndex + distortionIntrinsicDim;
end

if tdDim
    options.td = lastCameraParamIndex + 1;
    lastCameraParamIndex = lastCameraParamIndex + 1;
end

if trDim
    options.tr = lastCameraParamIndex + 1;
    lastCameraParamIndex = lastCameraParamIndex + 1;
end

% set indices for IMU parameters' std devs.
options.T_g_std = [];
options.T_s_std = [];
options.T_a_std = [];
lastParamStdIndex = options.std_start_index + 15;
if isfilter && imumodel == 1
    options.T_g_std = lastParamStdIndex + (1:9);
    options.T_s_std = lastParamStdIndex + (10:18);
    options.T_a_std = lastParamStdIndex + (19:24);
    lastParamStdIndex = lastParamStdIndex + 24;
end

% set indices for the chosen camera parameters' std devs.
options.p_camera_std = [];
options.q_XC_std = [];
options.fxy_cxy_std = [];
options.k1_k2_std = [];
options.p1_p2_std = [];
options.td_std = [];
options.tr_std = [];

if isfilter
    lastParamStdIndex = lastParamStdIndex + cameraParamMinDim * (plotCamIdx - 1);

    options.p_camera_std = lastParamStdIndex + (1:3);
    options.q_XC_std = lastParamStdIndex + (4:6);
    lastParamStdIndex = lastParamStdIndex + 6;

    if projectionIntrinsicDim
        options.fxy_cxy_std = lastParamStdIndex + (1:projectionIntrinsicDim);
        lastParamStdIndex = lastParamStdIndex + projectionIntrinsicDim;
    end

    if distortionIntrinsicDim
        options.k1_k2_std = lastParamStdIndex + (1:2);
        options.p1_p2_std = lastParamStdIndex + (3:distortionIntrinsicDim);
        lastParamStdIndex = lastParamStdIndex + distortionIntrinsicDim;
    end
    if tdDim
        options.td_std = lastParamStdIndex + 1;
        lastParamStdIndex = lastParamStdIndex + tdDim;
    end
    if trDim
        options.tr_std = lastParamStdIndex + 1;
        lastParamStdIndex = lastParamStdIndex + trDim;
    end
end
end
