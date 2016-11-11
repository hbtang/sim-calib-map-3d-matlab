function [ jacobian ] = diffRotVec2Lie( mat_C, vec_v )
% differentiate Cv with psi, where C is rotation matrix, v is 3d vector,
% psi is lie-algebra (also rodrigues vector)

vec_psi = rodrigues(mat_C);
psi = norm(vec_psi);
vec_a = vec_psi/psi;

J_psi = sin(psi)/psi*eye(3) + (1 - sin(psi)/psi)*(vec_a*vec_a.') + (1-cos(psi))/psi*skewsymm(vec_a);
jacobian = - skewSymm(mat_C*vec_v) * J_psi;

%% debug

% cost = mat_C*vec_v;
% delta = 1e-6;
% 
% vec_psi_p1 = vec_psi + delta*[1;0;0];
% mat_C_p1 = rodrigues(vec_psi_p1);
% cost_p1 = mat_C_p1 * vec_v;
% jacobian1_num = (cost_p1-cost)/delta;
% 
% vec_psi_p2 = vec_psi + delta*[0;1;0];
% mat_C_p2 = rodrigues(vec_psi_p2);
% cost_p2 = mat_C_p2 * vec_v;
% jacobian2_num = (cost_p2-cost)/delta;
% 
% vec_psi_p3 = vec_psi + delta*[0;0;1];
% mat_C_p3 = rodrigues(vec_psi_p3);
% cost_p3 = mat_C_p3 * vec_v;
% jacobian3_num = (cost_p3-cost)/delta;
% 
% jacobian_num = [jacobian1_num jacobian2_num jacobian3_num];
% jacobian_num - jacobian;

end

