function [ ps2d_w_b2 ] = FunMove2d( ps2d_w_b1, ps2d_b1_b2 )
%FUNMOVE2D Summary of this function goes here
%   Detailed explanation goes here

T2d_w_b1 = FunVec2Trans2d(ps2d_w_b1);
T2d_b1_b2 = FunVec2Trans2d(ps2d_b1_b2);

T2d_w_b2 = T2d_w_b1*T2d_b1_b2;

ps2d_w_b2 = FunTrans2Vec2d(T2d_w_b2);

end

