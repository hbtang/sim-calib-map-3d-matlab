function [ output ] = vec_matrix_2d( input )
%VEC_2_MATRIX_2D; change between 2d trans vec and 2d trans matrix
%   Detailed explanation goes here
if size(input) == [3 1]
    vec = input;
    theta = vec(3);
    x = vec(1);
    y = vec(2);
    R = [cos(theta) -sin(theta) x;
        sin(theta) cos(theta) y;
        0 0 1];
    output = R;
    return;
end
if size(input) == [3 3]
    R = input;
    theta = atan2(R(2,1),R(1,1));
    x = R(1,3);
    y = R(2,3);
    vec = [x;y;theta];
    output = vec;
    return;
end  
    
    
end

