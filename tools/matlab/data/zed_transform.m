% q, t express the pose of IMU frame in the left camera frame, obtained
% from zed2 ros log at the start or ros message left_cam_imu_transform.
q = quaternion(0.99999499321, 0.0, 0.0006103515625, -0.00309779727831);
t = [-0.00200000009499; -0.02300000377; 0.000220000030822];

% P2 K2 are from the right_camera camera_info.
P2 = [264.9825134277344, 0.0, 330.62750244140625, -31.662097930908203;
    0.0, 264.73748779296875, 188.26150512695312, 0.0; 
    0.0, 0.0, 1.0, 0.0];
K2 = [264.9825134277344, 0.0, 330.62750244140625;
    0.0, 264.73748779296875, 188.26150512695312;
    0.0, 0.0, 1.0];

R = quat2rotm(q);
T_Cp_IMU = [R, t; 0, 0, 0, 1];
T_C_Cp = [0, -1, 0, 0; 0, 0, -1, 0; 1, 0, 0, 0; 0, 0, 0, 1];
T_C_IMU = T_C_Cp * T_Cp_IMU;
format longg;
T_IMU_C1 = inv(T_C_IMU)

RT2 = inv(K2) * P2;
p_C2_C1 = RT2(:, 4);

T_C2_C1 = [RT2; 0, 0, 0, 1];
T_IMU_C2 = T_IMU_C1 * inv(T_C2_C1)

