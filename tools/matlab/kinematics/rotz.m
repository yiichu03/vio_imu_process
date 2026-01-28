function m = rotz(psi)
% following NCLT dataset coordinate frame convention.
c = cos(psi);
s = sin(psi);
m = [c, s, 0; -s, c, 0; 0, 0, 1];
end
