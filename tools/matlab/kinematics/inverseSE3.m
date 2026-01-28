function invdata = inverseSE3(data, pxyzIndices, qxyzwIndices)
invdata = zeros(size(data, 1), 7);
for i = 1:size(data, 1)
p = data(i, pxyzIndices);
q = data(i, qxyzwIndices);
qwxyz = quaternion([q(4), q(1:3)]);
pp = - rotatepoint(conj(qwxyz), p);
invdata(i, 1:3) = pp;
invdata(i, 4:7) = [-q(1:3), q(4)];
end
end
