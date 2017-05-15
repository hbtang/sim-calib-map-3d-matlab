function [ cov_ret ] = RefineCov( cov, s_min )
%COVPOSDEF refine covariance matrix
% keep a covariance matrix always positive definite
% and constraint on the smallest singular value

if nargin < 2
    s_min = 1e-6;
end

[num_row, num_col] = size(cov);
if num_row ~= num_col
    error('Error in CovPosDef!');
end

[U,S,V] = svd(cov);
S = max(S, eye(num_row)*s_min);
cov_ret = U*S*U.';

cov_ret = (cov_ret + cov_ret.')/2;

end

