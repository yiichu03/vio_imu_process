function m = rotxyz(euler)
% following NCLT dataset coordinate frame convention.
% euler = [phi, theta, psi].
m = rotx(euler(1)) * roty(euler(2)) * rotz(euler(3));
m = m';
end

