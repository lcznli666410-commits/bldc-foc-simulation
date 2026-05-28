function y = clarke_park_calc(u)
%% CLARKE_PARK_CALC Simulink用Clarke+Park变换函数
%  输入: u = [ia; ib; ic; theta_e]
%  输出: y = [id; iq]

    ia = u(1);
    ib = u(2);
    ic = u(3);
    theta_e = u(4);
    
    % Clarke变换
    i_alpha = (2/3) * (ia - 0.5*ib - 0.5*ic);
    i_beta  = (2/3) * (sqrt(3)/2*ib - sqrt(3)/2*ic);
    
    % Park变换
    id =  i_alpha * cos(theta_e) + i_beta * sin(theta_e);
    iq = -i_alpha * sin(theta_e) + i_beta * cos(theta_e);
    
    y = [id; iq];
end
