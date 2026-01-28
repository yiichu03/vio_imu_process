function data = loadOpenVinsResult(resultdir, runno)
% load openvins results and convert to errors.

est = readmatrix([resultdir, '/state_estimate_', num2str(runno), '.txt'], 'NumHeaderLines', 1);
std = readmatrix([resultdir, '/state_deviation_', num2str(runno), '.txt'], 'NumHeaderLines', 1);
truth = readmatrix([resultdir, '/state_groundtruth_', num2str(runno), '.txt'], 'NumHeaderLines', 1);

data = [est, std];
data(:, 1) = data(:, 1) - 100;
startstd = size(est, 2);

extrinsicp = 32:34;
extrinsicq = 28:31;
extrinsicpstd = (30:32) + startstd;
extrinsicqstd = (27:29) + startstd;
td = 18;
tdstd = 17 + startstd;

ba = 15:17;
bastd = (14:16) + startstd;
bg = 12:14;
bgstd = (11:13) + startstd;
q = 2:5;
qstd = (2:4) + startstd;
p = 6:8;
pstd = (5:7) + startstd;
v = 9:11;
vstd = (8:10) + startstd;

qerror = zeros(size(est, 1), 3);
for i = 1:size(est, 1)
    quat = conj(quaternion([data(1, q(4)), data(1, q(1:3))])) * quaternion([data(i, q(4)), data(i, q(1:3))]) * ...
        conj(conj(quaternion([truth(1, q(4)), truth(1, q(1:3))])) * quaternion([truth(i, q(4)), truth(i, q(1:3))]));
%     quat = quaternion([data(i, q(4)), data(i, q(1:3))]) * ...
%         conj(quaternion([truth(i, q(4)), truth(i, q(1:3))]));
    qerror(i, :) = fliplr(quat2eul(quat));
end

q_SC_ref = quaternion([0.50 -0.50 0.50 -0.50]);
R_SC_ref = quat2rotm(q_SC_ref);

extrinsiceulZYX = zeros(size(data, 1), 3);
for i = 1:size(est, 1)
    extrinsiceulZYX(i, :) = quat2eul(quaternion(data(i, extrinsicq)) * conj(quaternion(truth(i, extrinsicq))));
end
extrinsicerroreul = fliplr(extrinsiceulZYX);

extrinsicpstddata = data(:, extrinsicpstd);
for i = 1:size(est, 1)
    extrinsicpstddata(i, :) = data(i, extrinsicpstd) * R_SC_ref';
end

data(:, q(1:3)) = qerror;

data(:, extrinsicq(1:3)) = extrinsicerroreul;
data(:, extrinsicp) = (data(:, extrinsicp) - truth(:, extrinsicp)) * R_SC_ref';
data(:, extrinsicpstd) = data(:, extrinsicpstd) * R_SC_ref';

data(:, p) = data(:, p) - truth(:, p);
data(:, v) = data(:, v) - truth(:, v);

data(:, bg) = data(:, bg) - truth(:, bg);
data(:, ba) = data(:, ba) - truth(:, ba);

data(:, td) = data(:, td) - truth(:, td);

end
