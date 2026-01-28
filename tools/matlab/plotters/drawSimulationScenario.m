function drawSimulationScenario(trajectory_txt, landmark_txt, outputdir, scene, anchorIndex)

% scene: Dot, Motionless, Squircle, WavyCircle, Lemniscate, LineSegment
% landmark_txt = [data_dir, '/', scene, '_Points.txt'];
% trajectory_txt = [data_dir, '/', scene, '.txt'];

export_fig_path = '/media/jhuai/BackupPlus/jhuai/tools/export_fig';
addpath(export_fig_path);

if nargin < 5
    anchorIndex = 37;
end

% swift_vio simulation setup
r = 3:5;
q = 6:9;
% body frame FLU, camera frame forward motion
T_BC = [0, 0, 1, 0;
    -1, 0, 0, 0;
    0, -1, 0, 0;
    0, 0, 0, 1];

% openvins simulation setup
r = 2:4;
q = 5:8;
% body frame FLU, camera frame forward motion
T_BC = [0,-1, 0, -0.05;
    1, 0, 0,  0.00;
    0, 0, 1,  0.00;
    0, 0, 0,  1.00];

% openvins simulation camera pointing forward motion
T_BC = [0, 0, 1, -0.00;
        -1, 0, 0,  0.00;
        0, -1, 0,  0.00;
        0, 0, 0,  1.00];

addpath(export_fig_path);
landmarks = readmatrix(landmark_txt, 'NumHeaderLines', 1);
landmarks = landmarks(:, 2:4);
landmarks = pcdownsample(pointCloud(landmarks),'random', 0.3);
landmarks = landmarks.Location;
trajectory = readmatrix(trajectory_txt, 'NumHeaderLines', 1);

T_WB = eye(4);
T_WB(1:3, 4) = trajectory(anchorIndex, r)';
T_WB(1:3, 1:3) = rotmat(quaternion([trajectory(anchorIndex, q(4)), trajectory(anchorIndex, q(1:3))]), 'point');
T_WC = T_WB * T_BC;
t_WC = T_WC(1:3, 4);
R_WC = T_WC(1:3, 1:3);
close all;
figure;

plot3(trajectory(:, r(1)), trajectory(:, r(2)), ...
    trajectory(:, r(3)), '-k', 'LineWidth', 1); hold on;
plot3(landmarks(:, 1), landmarks(:, 2), landmarks(:, 3), 'xb');
w = 6.4;
h = 3.6;
f = 5.0;
scale = 0.6;
drawCameraFrustum(t_WC, R_WC, w, h, f, scale);

if strcmp(scene, 'Dot')
    [arc_point, arc_arrow] = arcWithArrow(5, -210 / 180 * pi, -60 / 180 * pi, 1);
    plot3(arc_point(1, :), arc_point(2, :), arc_point(3, :), 'b');
    plot3(arc_arrow(1, :), arc_arrow(2, :), arc_arrow(3, :), 'b', 'LineWidth', 1);
end
legend_list = {'trajectory', 'landmarks'};
% legend(legend_list);
% title([scene, ' p_{GB}']);
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
set(gcf, 'Color', 'None');
axis equal;
grid on;
% xlim([-5, 40]);
% ylim([-6.0, 6.0]);
% zlim([-5, 25]);
outputfig = [outputdir, '/', scene, '.pdf'];
if exist(outputfig, 'file')==2
  delete(outputfig);
end
export_fig(outputfig);

end
