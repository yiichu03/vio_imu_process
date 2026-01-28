% state [x1, x2]'
% Normal equation: [M, N] [x1; x2] = b
% H = [M, N] = [A, B; B', D]
% Effect of x2 on x1: - pinv(M) * N

m = 5; % dim of x1
n = 3; % dim of x2
d = m + n;
a = rand(d, d);
H = a * a';
S = H \ eye(d, d);
M = H(:, 1:m);
N = H(:, m+1:end);

pinvM = (transpose(M) * M) \ transpose(M);

absdiff = abs(pinvM * M - eye(m));
assert(all(absdiff(:) < 1e-8));

disp('The effect of state x2 on state x1: -M^(+) * N');
disp(-pinvM * N);

AinvB = - H(1:m, 1:m) \ H(1:m, m+1:end);
rho2 = S(1:m, m+1:end) / S(m+1:end, m+1:end);

absdiff = abs(rho2 - AinvB);
disp('cov(x1, x2) * cov(x2, x2)^(-1) == A \ B');
disp(rho2);
assert(all(absdiff(:) < 1e-8));

% conclusion: -pinvM * N is close to A \ B', perhaps because it considers
% additional effect from D.

disp('cov(x2, x1) * cov(x1, x1)^(-1) == D \ B');
DinvBt = - H(m+1:end, m+1:end) \ H(m+1:end, 1:m);
rho1 = S(m+1:end, 1:m) / S(1:m, 1:m);
absdiff = abs(rho1 - DinvBt);
assert(all(absdiff(:) < 1e-8));
% disp(rho1);

