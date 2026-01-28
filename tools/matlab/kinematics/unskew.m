function res=unskew(mx)
% unskew(mx) mx is the skew symmetric matrix of vector
% [1,2,3], i.e., 
% [ 0, -3, 2;
%  3, 0, -1;
% -2, 1, 0]
res=([mx(3,2);mx(1,3);mx(2,1)]-[mx(2,3);mx(3,1);mx(1,2)])/2;

