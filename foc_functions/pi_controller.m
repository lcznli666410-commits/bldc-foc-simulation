function [output, integrator] = pi_controller(error, Kp, Ki, dt, integrator, out_max, out_min)
%% PI_CONTROLLER 带抗饱和的PI控制器
%  实现带积分限幅和抗饱和功能的PI控制器
%
%  输入:
%    error      - 误差信号
%    Kp         - 比例增益
%    Ki         - 积分增益
%    dt         - 采样时间 [s]
%    integrator - 积分器状态 (上一步的值)
%    out_max    - 输出上限
%    out_min    - 输出下限
%
%  输出:
%    output     - 控制器输出
%    integrator - 更新后的积分器状态

    % 比例项
    P_term = Kp * error;
    
    % 积分项 (前向欧拉)
    integrator = integrator + Ki * error * dt;
    
    % 计算未限幅输出
    output_unsat = P_term + integrator;
    
    % 输出限幅
    output = max(out_min, min(out_max, output_unsat));
    
    % 抗饱和: 如果输出饱和，回退积分器
    if output ~= output_unsat
        integrator = integrator - Ki * error * dt;
    end
    
end
