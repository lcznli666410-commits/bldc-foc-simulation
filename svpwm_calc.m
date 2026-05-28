function y = svpwm_calc(u)
%% SVPWM_CALC Simulink用SVPWM函数
%  输入: u = [V_alpha; V_beta]
%  输出: y = [Va; Vb; Vc] 三相电压

    v_alpha = u(1);
    v_beta = u(2);
    
    % 使用工作空间中的Vdc参数
    Vdc = evalin('base', 'motor.Vdc');
    
    %% 扇区判断
    Vref1 = v_beta;
    Vref2 = (sqrt(3)/2 * v_alpha - 0.5 * v_beta);
    Vref3 = (-sqrt(3)/2 * v_alpha - 0.5 * v_beta);
    
    A = (Vref1 > 0);
    B = (Vref2 > 0);
    C = (Vref3 > 0);
    N = 4*C + 2*B + A;
    
    sector_map = [0, 2, 6, 1, 4, 3, 5, 0];
    sector = sector_map(N + 1);
    if sector == 0
        sector = 1;
    end
    
    %% 计算时间
    K = sqrt(3) / Vdc;
    
    switch sector
        case 1
            t1 = K * (sqrt(3)/2 * v_alpha - 0.5 * v_beta);
            t2 = K * v_beta;
        case 2
            t1 = K * (sqrt(3)/2 * v_alpha + 0.5 * v_beta);
            t2 = K * (-sqrt(3)/2 * v_alpha + 0.5 * v_beta);
        case 3
            t1 = K * v_beta;
            t2 = K * (-sqrt(3)/2 * v_alpha - 0.5 * v_beta);
        case 4
            t1 = -K * v_beta;
            t2 = K * (-sqrt(3)/2 * v_alpha + 0.5 * v_beta);
        case 5
            t1 = K * (-sqrt(3)/2 * v_alpha - 0.5 * v_beta);
            t2 = K * (sqrt(3)/2 * v_alpha - 0.5 * v_beta);
        case 6
            t1 = K * (-sqrt(3)/2 * v_alpha + 0.5 * v_beta);
            t2 = -K * v_beta;
        otherwise
            t1 = 0; t2 = 0;
    end
    
    if (t1 + t2) > 1
        t1 = t1 / (t1 + t2);
        t2 = t2 / (t1 + t2);
    end
    
    t0 = 1 - t1 - t2;
    
    %% 计算相电压
    switch sector
        case 1
            Ta = t0/2;         Tb = t0/2 + t1;      Tc = t0/2 + t1 + t2;
        case 2
            Ta = t0/2 + t2;    Tb = t0/2;            Tc = t0/2 + t1 + t2;
        case 3
            Ta = t0/2+t1+t2;   Tb = t0/2;            Tc = t0/2 + t1;
        case 4
            Ta = t0/2+t1+t2;   Tb = t0/2 + t2;       Tc = t0/2;
        case 5
            Ta = t0/2 + t1;    Tb = t0/2 + t1 + t2;  Tc = t0/2;
        case 6
            Ta = t0/2;         Tb = t0/2 + t1 + t2;  Tc = t0/2 + t2;
        otherwise
            Ta = 0.5; Tb = 0.5; Tc = 0.5;
    end
    
    % 转换为三相电压
    Va = (2*Ta - 1) * Vdc / 2;
    Vb = (2*Tb - 1) * Vdc / 2;
    Vc = (2*Tc - 1) * Vdc / 2;
    
    y = [Va; Vb; Vc];
end
