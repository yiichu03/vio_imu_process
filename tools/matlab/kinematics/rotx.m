function m = rotx(phi)
% following NCLT dataset coordinate frame convention.
c = cos(phi);
s = sin(phi);
m = [1, 0, 0; 0, c, s; 0, -s, c];
end
