function drawTrajWithFramesDemo(fn)
matrixList = cell(1, 1);
matrixList{1} = readmatrix(fn);
[p, n, e] = fileparts(fn);
[p2, n2, e2] = fileparts(p);
labels = cell(1, 1);
labels{1} = n;
txyzIndices = 2:4;
qxyzwIndices = 5:8;
figure;
lineHandles = drawTrajectoryWithCoordinateFrames(matrixList, labels, txyzIndices, ...
    qxyzwIndices);

plot3(matrixList{1}(1, 2), matrixList{1}(1, 3), matrixList{1}(1, 4), 'ko', 'MarkerSize', 10);
plot3(matrixList{1}(end, 2), matrixList{1}(end, 3), matrixList{1}(end, 4), 'rs', 'MarkerSize', 10);
title(n2, 'Interpreter', 'none');
end
