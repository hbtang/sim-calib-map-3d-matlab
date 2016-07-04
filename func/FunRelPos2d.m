function [ pose_i_j ] = FunRelPos2d( pose_i, pose_j )
%FUNRelPos2d calculate relative 2d pose w.r.t pose_i

theta_i = pose_i(3);
pose_i_j = [cos(-theta_i) -sin(-theta_i) 0;...
    sin(-theta_i) cos(-theta_i) 0;
    0 0 1]*(pose_j - pose_i);

pose_i_j(3) = cnstr2period( pose_i_j(3), pi, -pi );

end

