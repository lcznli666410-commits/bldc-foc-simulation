function [Ta, Tb, Tc, sector] = svpwm(v_alpha, v_beta, Vdc)
%% SVPWM 空间矢量脉宽调制
%  根据alpha-beta轴电压计算三相PWM占空比
%
%  输入:
%    v_alpha - alpha轴电压 [V]
%    v_beta  - beta轴电压 [V]
%    Vdc     - 直流母线电压 [V]
%
%  输出:
%    Ta, Tb, Tc - 三相占空比 [0~1]
%    sector     - 所在扇区 [1~6]

    %% 计算参考电压矢量在扇区判断用的变量
    Vref1 = v_beta;
    Vref2 = (sqrt(3)/2 * v_alpha - 0.5 * v_beta);
    Vref3 = (-sqrt(3)/2 * v_alpha - 0.5 * v_beta);
    
    %% 扇区判断
    % 使用符号判断法
    A = (Vref1 > 0);
    B = (Vref2 > 0);
    C = (Vref3 > 0);
    
    N = 4*C + 2*B + A;
    
    % 扇区映射
    sector_map = [0, 2, 6, 1, 4, 3, 5, 0];  % N -> sector
    sector = sector_map(N + 1);
    
    if sector == 0
        sector = 1;  % 默认扇区
    end
    
    %% 计算基本矢量作用时间
    % 归一化系数
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
            t1 = 0;
            t2 = 0;
    end
    
    %% 过调制处理
    if (t1 + t2) > 1
        t1 = t1 / (t1 + t2);
        t2 = t2 / (t1 + t2);
    end
    
    t0 = 1 - t1 - t2;  % 零矢量作用时间
    
    %% 计算三相切换时间
    % 七段式SVPWM
    switch sector
        case 1
            Ta_on = (t0/2);
            Tb_on = (t0/2 + t1);
            Tc_on = (t0/2 + t1 + t2);
        case 2
            Ta_on = (t0/2 + t2);
            Tb_on = (t0/2);
            Tc_on = (t0/2 + t1 + t2);
        case 3
            Ta_on = (t0/2 + t1 + t2);
            Tb_on = (t0/2);
            Tc_on = (t0/2 + t1);
        case 4
            Ta_on = (t0/2 + t1 + t2);
            Tb_on = (t0/2 + t2);
            Tc_on = (t0/2);
        case 5
            Ta_on = (t0/2 + t1);
            Tb_on = (t0/2 + t1 + t2);
            Tc_on = (t0/2);
        case 6
            Ta_on = (t0/2);
            Tb_on = (t0/2 + t1 + t2);
            Tc_on = (t0/2 + t2);
        otherwise
            Ta_on = 0.5;
            Tb_on = 0.5;
            Tc_on = 0.5;
    end
    
    %% 转换为占空比 (中心对齐)
    Ta = 1 - 2*Ta_on;
    Tb = 1 - 2*Tb_on;
    Tc = 1 - 2*Tc_on;
    
    % 限幅
    Ta = max(0, min(1, (Ta + 1) / 2));
    Tb = max(0, min(1, (Tb + 1) / 2));
    Tc = max(0, min(1, (Tc + 1) / 2));
    
end
