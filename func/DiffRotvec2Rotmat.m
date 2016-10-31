function [ dR1, dR2, dR3 ] = DiffRotvec2Rotmat( rvec )
%DIFFROTVEC2ROTMAT compute the differentiation of rodrigues vector to its
% rotation matrix
% return: Ri = dR/dv_i

% dcm: direct cosine matrix, rotation defined in matlab, transpose of the
% common definition of rotation matrix

R = rodrigues(rvec);
q = dcm2quat(R');

q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);

v = norm(rvec);
v1 = rvec(1); v2 = rvec(2); v3 = rvec(3);

if v < 1e-6
    dR1 = [0 0 0; 0 0 1; 0 -1 0];
    dR2 = [0 0 -1; 0 0 0; 1 0 0];
    dR3 = [0 1 0; -1 0 0; 0 0 0];
else
    cv2 = cos(v/2); sv2 = sin(v/2);
    a = cv2*v - 2*sv2;
    b = -sv2*v^2 - 6*cv2*v + 12*sv2;    
    
    
    % follow reference book "representing attitude ...", but maybe error in
    % F(), eq. 250-252, should times 2
    
    G = (1/(2*v^3)) * [ ...
        -v1*v^2*sv2         -v2*v^2*sv2         -v3*v^2*sv2;...
        2*v^2*sv2+v1^2*a    v1*v2*a             v1*v3*a;...
        v1*v2*a             2*v^2*sv2+v2^2*a    v2*v3*a;...
        v1*v3*a             v2*v3*a             2*v^2*sv2+v3^2*a...        
        ];
    
    g1 = G(:,1);
    g2 = G(:,2);
    g3 = G(:,3);
    
    F1 = 2*[...
        q0 q1 -q2 -q3;...
        -q3 q2 q1 -q0;...
        q2 q3 q0 q1;...
        ];
    
    F2 = 2*[...
        q3 q2 q1 q0;...
        q0 -q1 q2 -q3;...
        -q1 -q0 q3 q2;...
        ];
    
    F3 = 2*[...
        -q2 q3 -q0 q1;...
        q1 q0 q3 q2;...
        q0 -q1 -q2 q3;...
        ];
    
    dR1 = [F1*g1 F2*g1 F3*g1];
    dR2 = [F1*g2 F2*g2 F3*g2];
    dR3 = [F1*g3 F2*g3 F3*g3];
    
end

% transpose due to the definition of dcm in Matlab
dR1 = dR1';
dR2 = dR2';
dR3 = dR3';

%% debug: numerical as comparision
% R = rodrigues(rvec);
% 
% input1 = rvec; input1(1) = input1(1)+1e-6;
% R1 = rodrigues(input1);
% 
% input2 = rvec; input2(2) = input2(2)+1e-6;
% R2 = rodrigues(input2);
% 
% input3 = rvec; input3(3) = input3(3)+1e-6;
% R3 = rodrigues(input3);
% 
% dR1_num = (R1-R)/1e-6;
% dR2_num = (R2-R)/1e-6;
% dR3_num = (R3-R)/1e-6;
% 
% dR1_num - dR1
% dR2_num - dR2
% dR3_num - dR3

end

