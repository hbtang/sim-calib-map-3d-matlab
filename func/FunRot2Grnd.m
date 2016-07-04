function [ pvec_g_c,dist_g_c ] = FunRot2Grnd( rvec_cg_c, tvec_cg_c )
%FUNROT2GRND calculate the ground plane in camera from rotation_cg_c

if size(rvec_cg_c) == [3,1]
    R3d_cg_c = rodrigues(rvec_cg_c);
elseif size(rvec_cg_c) == [3,3]
    R3d_cg_c = rvec_cg_c;
else
    error('Error in FunRot2Grnd, input dimension error!');
end

pvec_g_c = R3d_cg_c.'*[0;0;1];

if nargout == 2
    % tvec_cg_c_c: tvec_cg_c w.r.t. frame c
    tvec_cg_c_c = R3d_cg_c.'*tvec_cg_c;
    dist_g_c = tvec_cg_c_c.'*pvec_g_c;    
end

end

