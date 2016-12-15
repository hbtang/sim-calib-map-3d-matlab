function [ rvec_a_b ] = drvec( rvec_a, rvec_b )
%DRVEC 此处显示有关此函数的摘要
%   此处显示详细说明
R3_a = rodrigues(rvec_a);
R3_b = rodrigues(rvec_b);

R3_a_b = R3_a.'*R3_b;
rvec_a_b = rodrigues(R3_a_b);

end

