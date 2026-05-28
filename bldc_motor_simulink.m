function y = bldc_motor_simulink(u)
%% BLDC_MOTOR_SIMULINK Simulink用BLDC电机模型
%  使用持久变量保存电机状态
%
%  输入: u = [Va; Vb; Vc; TL]
%  输出: y = [ia; ib; ic; theta_e; omega_m; Te]

    persistent ia ib ic omega_m theta_e initialized
    
    if isempty(initialized)
        ia = 0; ib = 0; ic = 0;
        omega_m = 0;
        theta_e = 0;
        initialized = true;
    end
    
    % 输入
    va = u(1);
    vb = u(2);
    vc = u(3);
    TL = u(4);
    
    % 获取参数
    Rs = evalin('base', 'motor.Rs');
    Ld = evalin('base', 'motor.Ld');
    Lq = evalin('base', 'motor.Lq');
    flux = evalin('base', 'motor.flux');
    Pp = evalin('base', 'motor.pole_pairs');
    J = evalin('base', 'motor.J');
    B = evalin('base', 'motor.B');
    dt = evalin('base', 'sim_params.Ts');
    
    % 电角速度
    omega_e = omega_m * Pp;
    
    % Clarke变换 - 电压
    v_alpha = (2/3) * (va - 0.5*vb - 0.5*vc);
    v_beta  = (2/3) * (sqrt(3)/2*vb - sqrt(3)/2*vc);
    
    % Park变换 - 电压
    vd =  v_alpha * cos(theta_e) + v_beta * sin(theta_e);
    vq = -v_alpha * sin(theta_e) + v_beta * cos(theta_e);
    
    % Clarke + Park - 电流
    i_alpha = (2/3) * (ia - 0.5*ib - 0.5*ic);
    i_beta  = (2/3) * (sqrt(3)/2*ib - sqrt(3)/2*ic);
    id =  i_alpha * cos(theta_e) + i_beta * sin(theta_e);
    iq = -i_alpha * sin(theta_e) + i_beta * cos(theta_e);
    
    % dq电压方程求解电流
    did_dt = (vd - Rs*id + omega_e*Lq*iq) / Ld;
    diq_dt = (vq - Rs*iq - omega_e*Ld*id - omega_e*flux) / Lq;
    
    id_new = id + did_dt * dt;
    iq_new = iq + diq_dt * dt;
    
    % 电磁转矩
    Te = 1.5 * Pp * (flux * iq_new + (Ld - Lq) * id_new * iq_new);
    
    % 机械方程
    domega_m = (Te - TL - B*omega_m) / J;
    omega_m = omega_m + domega_m * dt;
    
    % 电角度更新
    omega_e = omega_m * Pp;
    theta_e = theta_e + omega_e * dt;
    theta_e = mod(theta_e, 2*pi);
    
    % 反Park + 反Clarke - 更新三相电流
    i_alpha_new = id_new * cos(theta_e) - iq_new * sin(theta_e);
    i_beta_new  = id_new * sin(theta_e) + iq_new * cos(theta_e);
    
    ia = i_alpha_new;
    ib = -0.5*i_alpha_new + sqrt(3)/2*i_beta_new;
    ic = -0.5*i_alpha_new - sqrt(3)/2*i_beta_new;
    
    % 输出
    y = [ia; ib; ic; theta_e; omega_m; Te];
end
