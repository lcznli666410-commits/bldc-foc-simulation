function [id, iq] = park_transform(i_alpha, i_beta, theta_e)
%% PARK_TRANSFORM Park变换 (alpha-beta -> dq)
%  将两相静止坐标系下的电流变换到两相旋转坐标系
%
%  输入:
%    i_alpha - alpha轴电流 [A]
%    i_beta  - beta轴电流 [A]
%    theta_e - 电角度 [rad]
%
%  输出:
%    id - d轴电流 [A]
%    iq - q轴电流 [A]
%
%  变换矩阵:
%    [id]   [ cos(theta)  sin(theta)] [i_alpha]
%    [iq] = [-sin(theta)  cos(theta)] [i_beta ]

    id =  i_alpha * cos(theta_e) + i_beta * sin(theta_e);
    iq = -i_alpha * sin(theta_e) + i_beta * cos(theta_e);
    
end
