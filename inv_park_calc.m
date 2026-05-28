function y = inv_park_calc(u)
%% INV_PARK_CALC Simulink用反Park变换函数
%  输入: u = [Vd; Vq; theta_e]
%  输出: y = [V_alpha; V_beta]

    vd = u(1);
    vq = u(2);
    theta_e = u(3);
    
    v_alpha = vd * cos(theta_e) - vq * sin(theta_e);
    v_beta  = vd * sin(theta_e) + vq * cos(theta_e);
    
    y = [v_alpha; v_beta];
end
