%% =========================================================================
%  BLDC FOC控制 - 独立MATLAB仿真 (不需要Simulink)
%  完整的磁场定向控制仿真，包含速度环和电流环
%  =========================================================================
%  作者: Auto-generated
%  日期: 2024
%  描述: 基于MATLAB脚本的BLDC电机FOC控制仿真
%        包含: Clarke/Park变换, PI控制, SVPWM, 电机模型
%  =========================================================================

clear; clc; close all;

%% 添加函数路径
addpath('foc_functions');

%% 加载电机参数
motor_parameters;

%% 仿真配置
dt = sim_params.Ts;              % 仿真步长
Tend = sim_params.Tend;          % 仿真结束时间
N = round(Tend / dt);            % 总步数
t = (0:N-1) * dt;                % 时间向量

% 速度参考值 [rpm -> rad/s]
speed_ref_rpm = sim_params.speed_ref;
speed_ref_rad = speed_ref_rpm * 2*pi / 60;

% id参考值 (表贴式PMSM, id* = 0)
id_ref = 0;

%% 初始化状态变量
% 电机状态
ia = 0; ib = 0; ic = 0;
omega_m = 0;      % 机械角速度 [rad/s]
theta_e = 0;      % 电角度 [rad]

% PI控制器积分器
int_speed = 0;     % 速度环积分器
int_id = 0;        % d轴电流环积分器
int_iq = 0;        % q轴电流环积分器

%% 数据存储 (降采样存储以节省内存)
downsample = 10;   % 每10步存储一次
N_store = floor(N / downsample);
data.t         = zeros(1, N_store);
data.ia        = zeros(1, N_store);
data.ib        = zeros(1, N_store);
data.ic        = zeros(1, N_store);
data.id        = zeros(1, N_store);
data.iq        = zeros(1, N_store);
data.id_ref    = zeros(1, N_store);
data.iq_ref    = zeros(1, N_store);
data.vd        = zeros(1, N_store);
data.vq        = zeros(1, N_store);
data.speed_rpm = zeros(1, N_store);
data.speed_ref = zeros(1, N_store);
data.Te        = zeros(1, N_store);
data.TL        = zeros(1, N_store);
data.theta_e   = zeros(1, N_store);
data.sector    = zeros(1, N_store);
data.duty_a    = zeros(1, N_store);
data.duty_b    = zeros(1, N_store);
data.duty_c    = zeros(1, N_store);

store_idx = 0;

%% 控制环采样周期
Ts_current = pwm.Ts;           % 电流环周期 = PWM周期
Ts_speed = Ts_current * 10;    % 速度环周期 = 10倍电流环

current_counter = 0;
speed_counter = 0;

% 控制输出暂存
vd_out = 0;
vq_out = 0;
iq_ref = 0;

fprintf('开始仿真...\n');
fprintf('仿真步数: %d, 步长: %.2e s, 总时间: %.3f s\n', N, dt, Tend);

%% ===================================================================
%  主仿真循环
%  ===================================================================
tic;
for k = 1:N
    current_time = (k-1) * dt;
    
    %% 负载转矩阶跃
    if current_time >= sim_params.load_time
        TL = sim_params.load_torque;
    else
        TL = 0;
    end
    
    %% ============== 速度参考值生成 ==============
    % 斜坡启动 (0.05s内从0加速到目标速度)
    ramp_time = 0.05;
    if current_time < ramp_time
        speed_ref_current = speed_ref_rad * (current_time / ramp_time);
    else
        speed_ref_current = speed_ref_rad;
    end
    
    %% ============== FOC控制算法 ==============
    
    % --- 电流采样 (Clarke + Park) ---
    [i_alpha, i_beta] = clarke_transform(ia, ib, ic);
    [id_fb, iq_fb] = park_transform(i_alpha, i_beta, theta_e);
    
    % --- 速度环 (低速率) ---
    speed_counter = speed_counter + dt;
    if speed_counter >= Ts_speed
        speed_counter = 0;
        
        speed_error = speed_ref_current - omega_m;
        [iq_ref, int_speed] = pi_controller(speed_error, ...
            pid_speed.Kp, pid_speed.Ki, Ts_speed, int_speed, ...
            pid_speed.max, pid_speed.min);
    end
    
    % --- 电流环 (高速率) ---
    current_counter = current_counter + dt;
    if current_counter >= Ts_current
        current_counter = 0;
        
        % d轴电流控制
        id_error = id_ref - id_fb;
        [vd_out, int_id] = pi_controller(id_error, ...
            pid_id.Kp, pid_id.Ki, Ts_current, int_id, ...
            pid_id.max, pid_id.min);
        
        % q轴电流控制
        iq_error = iq_ref - iq_fb;
        [vq_out, int_iq] = pi_controller(iq_error, ...
            pid_iq.Kp, pid_iq.Ki, Ts_current, int_iq, ...
            pid_iq.max, pid_iq.min);
        
        % 前馈解耦 (可选，提高动态响应)
        omega_e = omega_m * motor.pole_pairs;
        vd_out = vd_out - omega_e * motor.Lq * iq_fb;
        vq_out = vq_out + omega_e * motor.Ld * id_fb + omega_e * motor.flux;
        
        % 电压限幅 (圆形限幅)
        Vmax = motor.Vdc / sqrt(3);
        Vmag = sqrt(vd_out^2 + vq_out^2);
        if Vmag > Vmax
            vd_out = vd_out * Vmax / Vmag;
            vq_out = vq_out * Vmax / Vmag;
        end
    end
    
    %% ============== 反变换与SVPWM ==============
    % 反Park变换
    [v_alpha, v_beta] = inv_park_transform(vd_out, vq_out, theta_e);
    
    % SVPWM生成PWM占空比
    [duty_a, duty_b, duty_c, sector] = svpwm(v_alpha, v_beta, motor.Vdc);
    
    % 从占空比计算相电压
    va = (2*duty_a - 1) * motor.Vdc / 2;
    vb = (2*duty_b - 1) * motor.Vdc / 2;
    vc = (2*duty_c - 1) * motor.Vdc / 2;
    
    %% ============== 电机模型更新 ==============
    [ia, ib, ic, theta_e, omega_m, Te, omega_e] = bldc_motor_model(...
        va, vb, vc, TL, ia, ib, ic, omega_m, theta_e, motor, dt);
    
    %% ============== 数据存储 ==============
    if mod(k, downsample) == 0
        store_idx = store_idx + 1;
        data.t(store_idx)         = current_time;
        data.ia(store_idx)        = ia;
        data.ib(store_idx)        = ib;
        data.ic(store_idx)        = ic;
        data.id(store_idx)        = id_fb;
        data.iq(store_idx)        = iq_fb;
        data.id_ref(store_idx)    = id_ref;
        data.iq_ref(store_idx)    = iq_ref;
        data.vd(store_idx)        = vd_out;
        data.vq(store_idx)        = vq_out;
        data.speed_rpm(store_idx) = omega_m * 60 / (2*pi);
        data.speed_ref(store_idx) = speed_ref_current * 60 / (2*pi);
        data.Te(store_idx)        = Te;
        data.TL(store_idx)        = TL;
        data.theta_e(store_idx)   = theta_e;
        data.sector(store_idx)    = sector;
        data.duty_a(store_idx)    = duty_a;
        data.duty_b(store_idx)    = duty_b;
        data.duty_c(store_idx)    = duty_c;
    end
    
    %% 进度显示
    if mod(k, round(N/10)) == 0
        fprintf('  进度: %.0f%%\n', k/N*100);
    end
end

elapsed = toc;
fprintf('仿真完成! 耗时: %.2f 秒\n', elapsed);

%% ===================================================================
%  结果可视化
%  ===================================================================

% 截取有效数据
idx = 1:store_idx;

figure('Name', 'BLDC FOC控制仿真结果', 'Position', [50 50 1400 900]);

%% 子图1: 转速响应
subplot(3,3,1);
plot(data.t(idx)*1000, data.speed_ref(idx), 'r--', 'LineWidth', 1.5); hold on;
plot(data.t(idx)*1000, data.speed_rpm(idx), 'b-', 'LineWidth', 1);
xlabel('时间 [ms]');
ylabel('转速 [rpm]');
title('转速响应');
legend('参考转速', '实际转速', 'Location', 'southeast');
grid on;

%% 子图2: d轴电流
subplot(3,3,2);
plot(data.t(idx)*1000, data.id_ref(idx), 'r--', 'LineWidth', 1.5); hold on;
plot(data.t(idx)*1000, data.id(idx), 'b-', 'LineWidth', 1);
xlabel('时间 [ms]');
ylabel('电流 [A]');
title('d轴电流');
legend('id*', 'id', 'Location', 'northeast');
grid on;

%% 子图3: q轴电流
subplot(3,3,3);
plot(data.t(idx)*1000, data.iq_ref(idx), 'r--', 'LineWidth', 1.5); hold on;
plot(data.t(idx)*1000, data.iq(idx), 'b-', 'LineWidth', 1);
xlabel('时间 [ms]');
ylabel('电流 [A]');
title('q轴电流');
legend('iq*', 'iq', 'Location', 'northeast');
grid on;

%% 子图4: 三相电流
subplot(3,3,4);
plot(data.t(idx)*1000, data.ia(idx), 'r-', 'LineWidth', 0.8); hold on;
plot(data.t(idx)*1000, data.ib(idx), 'g-', 'LineWidth', 0.8);
plot(data.t(idx)*1000, data.ic(idx), 'b-', 'LineWidth', 0.8);
xlabel('时间 [ms]');
ylabel('电流 [A]');
title('三相电流');
legend('ia', 'ib', 'ic', 'Location', 'northeast');
grid on;

%% 子图5: 电磁转矩
subplot(3,3,5);
plot(data.t(idx)*1000, data.Te(idx), 'b-', 'LineWidth', 1); hold on;
plot(data.t(idx)*1000, data.TL(idx), 'r--', 'LineWidth', 1.5);
xlabel('时间 [ms]');
ylabel('转矩 [Nm]');
title('电磁转矩与负载转矩');
legend('电磁转矩 Te', '负载转矩 TL', 'Location', 'northeast');
grid on;

%% 子图6: dq轴电压
subplot(3,3,6);
plot(data.t(idx)*1000, data.vd(idx), 'b-', 'LineWidth', 1); hold on;
plot(data.t(idx)*1000, data.vq(idx), 'r-', 'LineWidth', 1);
xlabel('时间 [ms]');
ylabel('电压 [V]');
title('dq轴电压');
legend('Vd', 'Vq', 'Location', 'northeast');
grid on;

%% 子图7: 电角度
subplot(3,3,7);
plot(data.t(idx)*1000, data.theta_e(idx)*180/pi, 'b-', 'LineWidth', 0.8);
xlabel('时间 [ms]');
ylabel('角度 [deg]');
title('电角度');
grid on;

%% 子图8: SVPWM扇区
subplot(3,3,8);
plot(data.t(idx)*1000, data.sector(idx), 'b.', 'MarkerSize', 1);
xlabel('时间 [ms]');
ylabel('扇区');
title('SVPWM扇区');
ylim([0.5 6.5]);
grid on;

%% 子图9: PWM占空比
subplot(3,3,9);
% 只显示最后一小段时间的占空比以看清PWM波形
t_start_ms = max(data.t(idx))*1000 - 5;  % 最后5ms
mask = data.t(idx)*1000 >= t_start_ms;
if sum(mask) > 10
    plot(data.t(mask)*1000, data.duty_a(mask), 'r-', 'LineWidth', 0.8); hold on;
    plot(data.t(mask)*1000, data.duty_b(mask), 'g-', 'LineWidth', 0.8);
    plot(data.t(mask)*1000, data.duty_c(mask), 'b-', 'LineWidth', 0.8);
    xlabel('时间 [ms]');
    ylabel('占空比');
    title('PWM占空比 (末段放大)');
    legend('A相', 'B相', 'C相', 'Location', 'best');
    ylim([-0.05 1.05]);
else
    plot(data.t(idx)*1000, data.duty_a(idx), 'r-', 'LineWidth', 0.8); hold on;
    plot(data.t(idx)*1000, data.duty_b(idx), 'g-', 'LineWidth', 0.8);
    plot(data.t(idx)*1000, data.duty_c(idx), 'b-', 'LineWidth', 0.8);
    xlabel('时间 [ms]');
    ylabel('占空比');
    title('PWM占空比');
    legend('A相', 'B相', 'C相', 'Location', 'best');
end
grid on;

sgtitle('BLDC电机 FOC矢量控制仿真结果', 'FontSize', 14, 'FontWeight', 'bold');

%% ===================================================================
%  额外分析图
%  ===================================================================

figure('Name', '电流矢量轨迹', 'Position', [100 100 800 400]);

% alpha-beta 电流轨迹
subplot(1,2,1);
[i_alpha_all, i_beta_all] = clarke_transform(data.ia(idx), data.ib(idx), data.ic(idx));
% 稳态段
steady_start = round(store_idx * 0.6);
plot(i_alpha_all(steady_start:end), i_beta_all(steady_start:end), 'b.', 'MarkerSize', 1);
xlabel('i_\alpha [A]');
ylabel('i_\beta [A]');
title('\alpha\beta 电流轨迹 (稳态)');
axis equal; grid on;

% dq电流轨迹
subplot(1,2,2);
plot(data.id(steady_start:store_idx), data.iq(steady_start:store_idx), 'r.', 'MarkerSize', 1);
xlabel('id [A]');
ylabel('iq [A]');
title('dq 电流轨迹 (稳态)');
axis equal; grid on;

sgtitle('电流矢量分析', 'FontSize', 14, 'FontWeight', 'bold');

%% 性能指标统计
fprintf('\n============================================\n');
fprintf('  仿真性能指标\n');
fprintf('============================================\n');

% 稳态段数据
ss_idx = round(store_idx*0.7):store_idx;
ss_before_load = (data.t > 0.15 & data.t < sim_params.load_time);

fprintf('  最终转速: %.1f rpm (参考: %.1f rpm)\n', ...
    data.speed_rpm(store_idx), speed_ref_rpm);
fprintf('  稳态速度误差: %.2f rpm\n', ...
    mean(abs(data.speed_rpm(ss_idx) - data.speed_ref(ss_idx))));
fprintf('  稳态id: %.4f A (参考: 0 A)\n', mean(data.id(ss_idx)));
fprintf('  稳态iq: %.4f A\n', mean(data.iq(ss_idx)));
fprintf('  稳态转矩: %.4f Nm\n', mean(data.Te(ss_idx)));
fprintf('  三相电流幅值(稳态): %.4f A\n', ...
    max(data.ia(ss_idx)));
fprintf('============================================\n');

fprintf('\n仿真结果已绘制完毕！\n');
fprintf('提示: 可以修改 motor_parameters.m 中的参数重新仿真\n');
