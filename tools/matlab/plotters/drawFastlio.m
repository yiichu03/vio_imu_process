% visualize the fastlio2 output relative to the ground truth for one NCLT
% dataset. The fastlio2 output uses the IMU as the body frame.
close all;
path = '/Downloads/fastlio/';
% The ground truth provided by NCLT dataset.
gteuler = readmatrix([path, 'groundtruth_2013-01-10.csv']);

% convert euler angles to hamilton quaternion.
gt = zeros(size(gteuler, 1), 8);
gt(:, 1:4) = gteuler(:, 1:4);
for i = 1:size(gt, 1)
    quat = rotm2quat(rotxyz(gteuler(i, 5:7))); % [w, x, y, z] 
    gt(i, 5:7) = quat(2:4);
    gt(i, 8) = quat(1);
end

% The fastlio2 output for the NCLT dataset.
est = readmatrix([path, 'stamped_traj_estimate0.txt']);
% Note fastlio2 converts the original IMU data with R_IN as
% indicated by https://drive.google.com/file/d/1leh7DxbHx29DyS1NJkvEfeNJoccxH7XM/view
R_IN = [0, -1, 0; -1, 0, 0; 0, 0, -1]; 
% I the IMU frame used in fastlio2, 
% N the NCLT dataset microstrain IMU frame.

% Convert the fastlio2 body frame to the NCLT body frame.
for i=1:size(est, 1)
    q_WI = est(i, 5:8);
    R_WN = quat2rotm([q_WI(4), q_WI(1:3)]) * R_IN; 
    q_WN = rotm2quat(R_WN);
    est(i, 5:7) = q_WN(2:4);
    est(i, 8) = q_WN(1);
end

rs = size(est, 1);
used = floor(rs / 5);
figure;
drawTrajectoryWithCoordinateFrames({est(1:used, :)}, {'est'}, 2:4, 5:8); hold on;
plot3(est(1, 2), est(1, 3), est(1, 4), 'ro', 'MarkerSize', 10);

figure;
drawTrajectoryWithCoordinateFrames({gt(1:20000, :)}, {'gt'}, 2:4, 5:8); hold on;
plot3(gt(1, 2), gt(1, 3), gt(1, 4), 'ro', 'MarkerSize', 10);
