function [v_alpha, v_beta] = inv_park_transform(vd, vq, theta_e)
%% INV_PARK_TRANSFORM 反Park变换 (dq -> alpha-beta)
%  将两相旋转坐标系下的电压变换到两相静止坐标系
%
%  输入:
%    vd      - d轴电压 [V]
%    vq      - q轴电压 [V]
%    theta_e - 电角度 [rad]
%
%  输出:
%    v_alpha - alpha轴电压 [V]
%    v_beta  - beta轴电压 [V]
%
%  变换矩阵:
%    [v_alpha]   [cos(theta)  -sin(theta)] [vd]
%    [v_beta ] = [sin(theta)   cos(theta)] [vq]

    v_alpha = vd * cos(theta_e) - vq * sin(theta_e);
    v_beta  = vd * sin(theta_e) + vq * cos(theta_e);
    
end
