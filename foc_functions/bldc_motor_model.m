function [ia, ib, ic, theta_e, omega_m, Te, omega_e] = bldc_motor_model(...
    va, vb, vc, TL, ia_prev, ib_prev, ic_prev, omega_m_prev, theta_e_prev, motor, dt)
%% BLDC_MOTOR_MODEL BLDC电机数学模型
%  基于dq轴模型的BLDC电机仿真
%
%  输入:
%    va, vb, vc   - 三相电压 [V]
%    TL            - 负载转矩 [Nm]
%    ia_prev, ib_prev, ic_prev - 上一步三相电流 [A]
%    omega_m_prev  - 上一步机械角速度 [rad/s]
%    theta_e_prev  - 上一步电角度 [rad]
%    motor         - 电机参数结构体
%    dt            - 仿真步长 [s]
%
%  输出:
%    ia, ib, ic   - 三相电流 [A]
%    theta_e      - 电角度 [rad]
%    omega_m      - 机械角速度 [rad/s]
%    Te           - 电磁转矩 [Nm]
%    omega_e      - 电角速度 [rad/s]

    %% 提取参数
    Rs = motor.Rs;
    Ld = motor.Ld;
    Lq = motor.Lq;
    flux = motor.flux;
    Pp = motor.pole_pairs;
    J = motor.J;
    B = motor.B;
    
    %% 电角速度
    omega_e_prev = omega_m_prev * Pp;
    
    %% Clarke变换 - 电压
    [v_alpha, v_beta] = clarke_transform(va, vb, vc);
    
    %% Park变换 - 电压和电流
    [vd, vq] = park_transform(v_alpha, v_beta, theta_e_prev);
    
    [i_alpha_prev, i_beta_prev] = clarke_transform(ia_prev, ib_prev, ic_prev);
    [id_prev, iq_prev] = park_transform(i_alpha_prev, i_beta_prev, theta_e_prev);
    
    %% dq轴电压方程 (求解电流)
    % vd = Rs*id + Ld*did/dt - omega_e*Lq*iq
    % vq = Rs*iq + Lq*diq/dt + omega_e*Ld*id + omega_e*flux
    
    did_dt = (vd - Rs*id_prev + omega_e_prev*Lq*iq_prev) / Ld;
    diq_dt = (vq - Rs*iq_prev - omega_e_prev*Ld*id_prev - omega_e_prev*flux) / Lq;
    
    % 欧拉积分
    id = id_prev + did_dt * dt;
    iq = iq_prev + diq_dt * dt;
    
    %% 电磁转矩计算
    % Te = 3/2 * Pp * (flux*iq + (Ld - Lq)*id*iq)
    Te = 1.5 * Pp * (flux * iq + (Ld - Lq) * id * iq);
    
    %% 机械方程
    % J*d(omega_m)/dt = Te - TL - B*omega_m
    domega_m_dt = (Te - TL - B*omega_m_prev) / J;
    omega_m = omega_m_prev + domega_m_dt * dt;
    
    %% 电角度更新
    omega_e = omega_m * Pp;
    theta_e = theta_e_prev + omega_e * dt;
    
    % 限制电角度在 [0, 2*pi]
    theta_e = mod(theta_e, 2*pi);
    
    %% 反Park变换 - 电流
    [i_alpha, i_beta] = inv_park_transform(id, iq, theta_e);
    
    %% 反Clarke变换 - 电流
    [ia, ib, ic] = inv_clarke_transform(i_alpha, i_beta);
    
end
