function lineHandles = drawTrajectoryWithCoordinateFrames(matrixList, labels, txyzIndices, ...
    qxyzwIndices)
% draw a trajectory and tips of local coordinate frames at specific 
% locations
% matrixList a cell of matrices, each row of each matrix containing a pose
% of format like (time, tx, ty, tz, qx, qy, qz, qw).
% at most four matrices are supported.
% labels are cell array 1xn for each matrix.
% txyzIndices indices of txyz in a row.
% qxyzwIndices indices of qxyzw in a row.

if nargin < 4
    qxyzwIndices = 5:8;
end
if nargin < 3
    txyzIndices = 2:4;
end
colors = {'r', 'g', 'b', 'k'};

count = min(length(matrixList), 4);
lineHandles = zeros(1, count);
range = max(max(matrixList{1}(:, txyzIndices))) - min(min(matrixList{1}(:, txyzIndices)));
scale = range / 50;

for j = 1:count
    lineHandles(j) = drawColumnsInMatrix(matrixList{j}, txyzIndices, 1, ...
        1, colors(j));
    hold on;
    maxCoordinateFrames = min(300, size(matrixList{j}, 1) / 3);
    increment = floor(size(matrixList{j}, 1) / maxCoordinateFrames);
    for i = 1:increment:size(matrixList{j}, 1)
        rot = rotmat(quaternion(matrixList{j}(i, [qxyzwIndices(4), ...
            qxyzwIndices(1:3)])), 'point');
        drawCoordinateFrame(matrixList{j}(i, txyzIndices)', rot, scale);
    end
end
legend(lineHandles, labels);
end
