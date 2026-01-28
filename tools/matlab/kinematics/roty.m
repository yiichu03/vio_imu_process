function m = roty(theta)
% following NCLT dataset coordinate frame convention.
c = cos(theta);
s = sin(theta);
m = [c, 0, -s; 0, 1, 0; s, 0, c];
end
