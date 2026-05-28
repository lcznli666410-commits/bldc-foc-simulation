%% =========================================================================
%  BLDC FOC控制 - Simulink模型自动构建脚本
%  使用MATLAB命令自动创建完整的Simulink模型
%  =========================================================================
%  运行此脚本将自动创建一个名为 'BLDC_FOC_Model' 的Simulink模型
%  包含: 电机模型, FOC控制器, SVPWM, 速度/电流环PI控制
%  =========================================================================

clear; clc; close all;

%% 加载电机参数
motor_parameters;

%% 模型名称
modelName = 'BLDC_FOC_Model';

% 如果模型已存在，关闭并删除
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if exist([modelName '.slx'], 'file')
    delete([modelName '.slx']);
end

%% ===================================================================
%  创建新的Simulink模型
%  ===================================================================
new_system(modelName);
open_system(modelName);

% 设置求解器参数
set_param(modelName, 'Solver', 'ode4');                  % 固定步长 Runge-Kutta
set_param(modelName, 'FixedStep', num2str(sim_params.Ts));
set_param(modelName, 'StopTime', num2str(sim_params.Tend));
set_param(modelName, 'SaveOutput', 'on');
set_param(modelName, 'SaveFormat', 'StructureWithTime');

fprintf('创建Simulink模型: %s\n', modelName);

%% 辅助函数: 安全清除子系统默认内容
% 不同版本的MATLAB/Simulink中SubSystem的默认内容不同
clear_subsystem_defaults = @(subsys) safe_clear_subsys(subsys);

%% ===================================================================
%  添加子系统
%  ===================================================================

%% -------------- 1. 速度参考值生成 子系统 ----------------
subsys_ref = [modelName '/Speed_Reference'];
add_block('built-in/SubSystem', subsys_ref);
set_param(subsys_ref, 'Position', [50 200 180 280]);

% 删除默认端口 (安全处理)
clear_subsystem_defaults(subsys_ref);

% 添加阶跃信号 (速度参考)
add_block('built-in/Step', [subsys_ref '/Speed_Step']);
set_param([subsys_ref '/Speed_Step'], ...
    'Time', '0.01', ...
    'Before', '0', ...
    'After', num2str(sim_params.speed_ref * 2*pi/60), ...
    'Position', [30 30 80 60]);

% 添加速率限制器 (斜坡启动)
add_block('built-in/RateLimiter', [subsys_ref '/Rate_Limiter']);
set_param([subsys_ref '/Rate_Limiter'], ...
    'RisingSlewLimit', num2str(sim_params.speed_ref * 2*pi/60 / 0.05), ...
    'FallingSlewLimit', num2str(-sim_params.speed_ref * 2*pi/60 / 0.05), ...
    'Position', [130 30 180 60]);

% 输出端口
add_block('built-in/Outport', [subsys_ref '/w_ref']);
set_param([subsys_ref '/w_ref'], 'Position', [230 38 260 52]);

add_line(subsys_ref, 'Speed_Step/1', 'Rate_Limiter/1');
add_line(subsys_ref, 'Rate_Limiter/1', 'w_ref/1');

fprintf('  [1/8] 速度参考值子系统 - 完成\n');

%% -------------- 2. 速度控制器 子系统 ----------------
subsys_speed = [modelName '/Speed_Controller'];
add_block('built-in/SubSystem', subsys_speed);
set_param(subsys_speed, 'Position', [250 200 400 300]);

clear_subsystem_defaults(subsys_speed);

% 输入端口
add_block('built-in/Inport', [subsys_speed '/w_ref'], 'Position', [30 38 60 52]);
add_block('built-in/Inport', [subsys_speed '/w_fb'], 'Position', [30 108 60 122]);

% 速度误差
add_block('built-in/Sum', [subsys_speed '/Sum_Speed']);
set_param([subsys_speed '/Sum_Speed'], 'Inputs', '+-', 'Position', [110 55 140 105]);

% PI控制器
add_block('built-in/TransferFcn', [subsys_speed '/PI_Speed']);
set_param([subsys_speed '/PI_Speed'], ...
    'Numerator', ['[' num2str(pid_speed.Kp) ' ' num2str(pid_speed.Ki) ']'], ...
    'Denominator', '[1 0]', ...
    'Position', [180 55 260 95]);

% 饱和限幅
add_block('built-in/Saturation', [subsys_speed '/Sat_Speed']);
set_param([subsys_speed '/Sat_Speed'], ...
    'UpperLimit', num2str(pid_speed.max), ...
    'LowerLimit', num2str(pid_speed.min), ...
    'Position', [300 60 340 90]);

% 输出 (iq_ref)
add_block('built-in/Outport', [subsys_speed '/iq_ref'], 'Position', [380 68 410 82]);

% id_ref 常数
add_block('built-in/Constant', [subsys_speed '/id_ref_const']);
set_param([subsys_speed '/id_ref_const'], 'Value', '0', 'Position', [300 140 340 170]);
add_block('built-in/Outport', [subsys_speed '/id_ref'], 'Position', [380 148 410 162]);

add_line(subsys_speed, 'w_ref/1', 'Sum_Speed/1');
add_line(subsys_speed, 'w_fb/1', 'Sum_Speed/2');
add_line(subsys_speed, 'Sum_Speed/1', 'PI_Speed/1');
add_line(subsys_speed, 'PI_Speed/1', 'Sat_Speed/1');
add_line(subsys_speed, 'Sat_Speed/1', 'iq_ref/1');
add_line(subsys_speed, 'id_ref_const/1', 'id_ref/1');

fprintf('  [2/8] 速度控制器子系统 - 完成\n');

%% -------------- 3. 电流控制器 子系统 ----------------
subsys_curr = [modelName '/Current_Controller'];
add_block('built-in/SubSystem', subsys_curr);
set_param(subsys_curr, 'Position', [450 180 620 340]);

clear_subsystem_defaults(subsys_curr);

% 输入端口
add_block('built-in/Inport', [subsys_curr '/id_ref'], 'Position', [30 38 60 52]);
add_block('built-in/Inport', [subsys_curr '/iq_ref'], 'Position', [30 108 60 122]);
add_block('built-in/Inport', [subsys_curr '/id_fb'],  'Position', [30 178 60 192]);
add_block('built-in/Inport', [subsys_curr '/iq_fb'],  'Position', [30 248 60 262]);

% d轴误差和PI
add_block('built-in/Sum', [subsys_curr '/Sum_id']);
set_param([subsys_curr '/Sum_id'], 'Inputs', '+-', 'Position', [120 25 150 75]);
add_block('built-in/TransferFcn', [subsys_curr '/PI_id']);
set_param([subsys_curr '/PI_id'], ...
    'Numerator', ['[' num2str(pid_id.Kp) ' ' num2str(pid_id.Ki) ']'], ...
    'Denominator', '[1 0]', ...
    'Position', [190 30 270 65]);
add_block('built-in/Saturation', [subsys_curr '/Sat_id']);
set_param([subsys_curr '/Sat_id'], ...
    'UpperLimit', num2str(pid_id.max), ...
    'LowerLimit', num2str(pid_id.min), ...
    'Position', [310 35 350 60]);

% q轴误差和PI
add_block('built-in/Sum', [subsys_curr '/Sum_iq']);
set_param([subsys_curr '/Sum_iq'], 'Inputs', '+-', 'Position', [120 140 150 190]);
add_block('built-in/TransferFcn', [subsys_curr '/PI_iq']);
set_param([subsys_curr '/PI_iq'], ...
    'Numerator', ['[' num2str(pid_iq.Kp) ' ' num2str(pid_iq.Ki) ']'], ...
    'Denominator', '[1 0]', ...
    'Position', [190 145 270 180]);
add_block('built-in/Saturation', [subsys_curr '/Sat_iq']);
set_param([subsys_curr '/Sat_iq'], ...
    'UpperLimit', num2str(pid_iq.max), ...
    'LowerLimit', num2str(pid_iq.min), ...
    'Position', [310 150 350 175]);

% 输出端口
add_block('built-in/Outport', [subsys_curr '/Vd'], 'Position', [400 42 430 58]);
add_block('built-in/Outport', [subsys_curr '/Vq'], 'Position', [400 157 430 168]);

% 连线
add_line(subsys_curr, 'id_ref/1', 'Sum_id/1');
add_line(subsys_curr, 'id_fb/1', 'Sum_id/2');
add_line(subsys_curr, 'Sum_id/1', 'PI_id/1');
add_line(subsys_curr, 'PI_id/1', 'Sat_id/1');
add_line(subsys_curr, 'Sat_id/1', 'Vd/1');

add_line(subsys_curr, 'iq_ref/1', 'Sum_iq/1');
add_line(subsys_curr, 'iq_fb/1', 'Sum_iq/2');
add_line(subsys_curr, 'Sum_iq/1', 'PI_iq/1');
add_line(subsys_curr, 'PI_iq/1', 'Sat_iq/1');
add_line(subsys_curr, 'Sat_iq/1', 'Vq/1');

fprintf('  [3/8] 电流控制器子系统 - 完成\n');

%% -------------- 4. 反Park变换 子系统 ----------------
subsys_ipark = [modelName '/Inv_Park'];
add_block('built-in/SubSystem', subsys_ipark);
set_param(subsys_ipark, 'Position', [680 200 830 310]);

clear_subsystem_defaults(subsys_ipark);

% 输入
add_block('built-in/Inport', [subsys_ipark '/Vd'], 'Position', [30 38 60 52]);
add_block('built-in/Inport', [subsys_ipark '/Vq'], 'Position', [30 88 60 102]);
add_block('built-in/Inport', [subsys_ipark '/theta_e'], 'Position', [30 138 60 152]);

% MATLAB函数实现反Park变换
add_block('built-in/MATLABFcn', [subsys_ipark '/InvPark_Fcn']);
set_param([subsys_ipark '/InvPark_Fcn'], ...
    'MATLABFcn', 'inv_park_calc', ...
    'Position', [150 50 250 120]);

% Mux输入
add_block('built-in/Mux', [subsys_ipark '/Mux']);
set_param([subsys_ipark '/Mux'], 'Inputs', '3', 'Position', [100 30 105 160]);

% Demux输出
add_block('built-in/Demux', [subsys_ipark '/Demux']);
set_param([subsys_ipark '/Demux'], 'Outputs', '2', 'Position', [290 50 295 120]);

% 输出
add_block('built-in/Outport', [subsys_ipark '/V_alpha'], 'Position', [340 58 370 72]);
add_block('built-in/Outport', [subsys_ipark '/V_beta'], 'Position', [340 98 370 112]);

add_line(subsys_ipark, 'Vd/1', 'Mux/1');
add_line(subsys_ipark, 'Vq/1', 'Mux/2');
add_line(subsys_ipark, 'theta_e/1', 'Mux/3');
add_line(subsys_ipark, 'Mux/1', 'InvPark_Fcn/1');
add_line(subsys_ipark, 'InvPark_Fcn/1', 'Demux/1');
add_line(subsys_ipark, 'Demux/1', 'V_alpha/1');
add_line(subsys_ipark, 'Demux/2', 'V_beta/1');

fprintf('  [4/8] 反Park变换子系统 - 完成\n');

%% -------------- 5. SVPWM 子系统 ----------------
subsys_svpwm = [modelName '/SVPWM'];
add_block('built-in/SubSystem', subsys_svpwm);
set_param(subsys_svpwm, 'Position', [900 200 1050 310]);

clear_subsystem_defaults(subsys_svpwm);

% 输入
add_block('built-in/Inport', [subsys_svpwm '/V_alpha'], 'Position', [30 38 60 52]);
add_block('built-in/Inport', [subsys_svpwm '/V_beta'], 'Position', [30 88 60 102]);

% Mux
add_block('built-in/Mux', [subsys_svpwm '/Mux']);
set_param([subsys_svpwm '/Mux'], 'Inputs', '2', 'Position', [100 30 105 110]);

% MATLAB函数
add_block('built-in/MATLABFcn', [subsys_svpwm '/SVPWM_Fcn']);
set_param([subsys_svpwm '/SVPWM_Fcn'], ...
    'MATLABFcn', 'svpwm_calc', ...
    'Position', [150 45 250 95]);

% Demux
add_block('built-in/Demux', [subsys_svpwm '/Demux']);
set_param([subsys_svpwm '/Demux'], 'Outputs', '3', 'Position', [290 30 295 110]);

% 输出
add_block('built-in/Outport', [subsys_svpwm '/Va'], 'Position', [340 28 370 42]);
add_block('built-in/Outport', [subsys_svpwm '/Vb'], 'Position', [340 63 370 77]);
add_block('built-in/Outport', [subsys_svpwm '/Vc'], 'Position', [340 98 370 112]);

add_line(subsys_svpwm, 'V_alpha/1', 'Mux/1');
add_line(subsys_svpwm, 'V_beta/1', 'Mux/2');
add_line(subsys_svpwm, 'Mux/1', 'SVPWM_Fcn/1');
add_line(subsys_svpwm, 'SVPWM_Fcn/1', 'Demux/1');
add_line(subsys_svpwm, 'Demux/1', 'Va/1');
add_line(subsys_svpwm, 'Demux/2', 'Vb/1');
add_line(subsys_svpwm, 'Demux/3', 'Vc/1');

fprintf('  [5/8] SVPWM子系统 - 完成\n');

%% -------------- 6. BLDC电机模型 子系统 ----------------
subsys_motor = [modelName '/BLDC_Motor'];
add_block('built-in/SubSystem', subsys_motor);
set_param(subsys_motor, 'Position', [1120 150 1350 400]);

clear_subsystem_defaults(subsys_motor);

% 输入端口
add_block('built-in/Inport', [subsys_motor '/Va'], 'Position', [30 28 60 42]);
add_block('built-in/Inport', [subsys_motor '/Vb'], 'Position', [30 78 60 92]);
add_block('built-in/Inport', [subsys_motor '/Vc'], 'Position', [30 128 60 142]);
add_block('built-in/Inport', [subsys_motor '/TL'], 'Position', [30 178 60 192]);

% Mux输入
add_block('built-in/Mux', [subsys_motor '/Mux_in']);
set_param([subsys_motor '/Mux_in'], 'Inputs', '4', 'Position', [100 20 105 200]);

% MATLAB函数 - 电机模型
add_block('built-in/MATLABFcn', [subsys_motor '/Motor_Fcn']);
set_param([subsys_motor '/Motor_Fcn'], ...
    'MATLABFcn', 'bldc_motor_simulink', ...
    'Position', [150 70 280 150]);

% Demux输出
add_block('built-in/Demux', [subsys_motor '/Demux_out']);
set_param([subsys_motor '/Demux_out'], 'Outputs', '6', 'Position', [320 20 325 250]);

% 输出端口
add_block('built-in/Outport', [subsys_motor '/ia'], 'Position', [380 18 410 32]);
add_block('built-in/Outport', [subsys_motor '/ib'], 'Position', [380 58 410 72]);
add_block('built-in/Outport', [subsys_motor '/ic'], 'Position', [380 98 410 112]);
add_block('built-in/Outport', [subsys_motor '/theta_e'], 'Position', [380 138 410 152]);
add_block('built-in/Outport', [subsys_motor '/omega_m'], 'Position', [380 178 410 192]);
add_block('built-in/Outport', [subsys_motor '/Te'], 'Position', [380 218 410 232]);

add_line(subsys_motor, 'Va/1', 'Mux_in/1');
add_line(subsys_motor, 'Vb/1', 'Mux_in/2');
add_line(subsys_motor, 'Vc/1', 'Mux_in/3');
add_line(subsys_motor, 'TL/1', 'Mux_in/4');
add_line(subsys_motor, 'Mux_in/1', 'Motor_Fcn/1');
add_line(subsys_motor, 'Motor_Fcn/1', 'Demux_out/1');
add_line(subsys_motor, 'Demux_out/1', 'ia/1');
add_line(subsys_motor, 'Demux_out/2', 'ib/1');
add_line(subsys_motor, 'Demux_out/3', 'ic/1');
add_line(subsys_motor, 'Demux_out/4', 'theta_e/1');
add_line(subsys_motor, 'Demux_out/5', 'omega_m/1');
add_line(subsys_motor, 'Demux_out/6', 'Te/1');

fprintf('  [6/8] BLDC电机模型子系统 - 完成\n');

%% -------------- 7. Park变换 (反馈) 子系统 ----------------
subsys_park = [modelName '/Park_Transform'];
add_block('built-in/SubSystem', subsys_park);
set_param(subsys_park, 'Position', [680 420 830 530]);

clear_subsystem_defaults(subsys_park);

% 输入
add_block('built-in/Inport', [subsys_park '/ia'], 'Position', [30 28 60 42]);
add_block('built-in/Inport', [subsys_park '/ib'], 'Position', [30 68 60 82]);
add_block('built-in/Inport', [subsys_park '/ic'], 'Position', [30 108 60 122]);
add_block('built-in/Inport', [subsys_park '/theta_e'], 'Position', [30 148 60 162]);

% Mux
add_block('built-in/Mux', [subsys_park '/Mux']);
set_param([subsys_park '/Mux'], 'Inputs', '4', 'Position', [100 20 105 170]);

% MATLAB函数
add_block('built-in/MATLABFcn', [subsys_park '/Park_Fcn']);
set_param([subsys_park '/Park_Fcn'], ...
    'MATLABFcn', 'clarke_park_calc', ...
    'Position', [150 60 250 130]);

% Demux
add_block('built-in/Demux', [subsys_park '/Demux']);
set_param([subsys_park '/Demux'], 'Outputs', '2', 'Position', [290 60 295 130]);

% 输出
add_block('built-in/Outport', [subsys_park '/id'], 'Position', [340 68 370 82]);
add_block('built-in/Outport', [subsys_park '/iq'], 'Position', [340 108 370 122]);

add_line(subsys_park, 'ia/1', 'Mux/1');
add_line(subsys_park, 'ib/1', 'Mux/2');
add_line(subsys_park, 'ic/1', 'Mux/3');
add_line(subsys_park, 'theta_e/1', 'Mux/4');
add_line(subsys_park, 'Mux/1', 'Park_Fcn/1');
add_line(subsys_park, 'Park_Fcn/1', 'Demux/1');
add_line(subsys_park, 'Demux/1', 'id/1');
add_line(subsys_park, 'Demux/2', 'iq/1');

fprintf('  [7/8] Park变换子系统 - 完成\n');

%% -------------- 8. 负载转矩输入 ----------------
add_block('built-in/Step', [modelName '/Load_Torque']);
set_param([modelName '/Load_Torque'], ...
    'Time', num2str(sim_params.load_time), ...
    'Before', '0', ...
    'After', num2str(sim_params.load_torque), ...
    'Position', [1000 380 1050 410]);

fprintf('  [8/8] 负载转矩输入 - 完成\n');

%% -------------- 添加示波器 (Scope) ----------------

% 速度示波器
add_block('built-in/Scope', [modelName '/Speed_Scope']);
set_param([modelName '/Speed_Scope'], ...
    'NumInputPorts', '2', ...
    'Position', [300 400 340 440]);

% 电流示波器
add_block('built-in/Scope', [modelName '/Current_dq_Scope']);
set_param([modelName '/Current_dq_Scope'], ...
    'NumInputPorts', '4', ...
    'Position', [500 420 540 500]);

% 转矩示波器
add_block('built-in/Scope', [modelName '/Torque_Scope']);
set_param([modelName '/Torque_Scope'], ...
    'NumInputPorts', '1', ...
    'Position', [1400 280 1440 320]);

% 三相电流示波器
add_block('built-in/Scope', [modelName '/Phase_Current_Scope']);
set_param([modelName '/Phase_Current_Scope'], ...
    'NumInputPorts', '3', ...
    'Position', [1400 150 1440 200]);

%% -------------- 添加 To Workspace ----------------
% 速度数据
add_block('built-in/ToWorkspace', [modelName '/speed_data']);
set_param([modelName '/speed_data'], ...
    'VariableName', 'speed_data', ...
    'Position', [300 470 380 500]);

%% ===================================================================
%  连接顶层模块
%  ===================================================================
fprintf('\n连接顶层信号线...\n');

% 速度参考 -> 速度控制器
add_line(modelName, 'Speed_Reference/1', 'Speed_Controller/1', 'autorouting', 'smart');

% 速度控制器 -> 电流控制器
add_line(modelName, 'Speed_Controller/2', 'Current_Controller/1', 'autorouting', 'smart');  % id_ref
add_line(modelName, 'Speed_Controller/1', 'Current_Controller/2', 'autorouting', 'smart');  % iq_ref

% 电流控制器 -> 反Park变换
add_line(modelName, 'Current_Controller/1', 'Inv_Park/1', 'autorouting', 'smart');  % Vd
add_line(modelName, 'Current_Controller/2', 'Inv_Park/2', 'autorouting', 'smart');  % Vq

% 反Park变换 -> SVPWM
add_line(modelName, 'Inv_Park/1', 'SVPWM/1', 'autorouting', 'smart');  % V_alpha
add_line(modelName, 'Inv_Park/2', 'SVPWM/2', 'autorouting', 'smart');  % V_beta

% SVPWM -> 电机
add_line(modelName, 'SVPWM/1', 'BLDC_Motor/1', 'autorouting', 'smart');  % Va
add_line(modelName, 'SVPWM/2', 'BLDC_Motor/2', 'autorouting', 'smart');  % Vb
add_line(modelName, 'SVPWM/3', 'BLDC_Motor/3', 'autorouting', 'smart');  % Vc

% 负载转矩 -> 电机
add_line(modelName, 'Load_Torque/1', 'BLDC_Motor/4', 'autorouting', 'smart');

% 电机反馈 -> Park变换
add_line(modelName, 'BLDC_Motor/1', 'Park_Transform/1', 'autorouting', 'smart');  % ia
add_line(modelName, 'BLDC_Motor/2', 'Park_Transform/2', 'autorouting', 'smart');  % ib
add_line(modelName, 'BLDC_Motor/3', 'Park_Transform/3', 'autorouting', 'smart');  % ic
add_line(modelName, 'BLDC_Motor/4', 'Park_Transform/4', 'autorouting', 'smart');  % theta_e

% theta_e -> 反Park变换
add_line(modelName, 'BLDC_Motor/4', 'Inv_Park/3', 'autorouting', 'smart');

% Park变换 -> 电流控制器反馈
add_line(modelName, 'Park_Transform/1', 'Current_Controller/3', 'autorouting', 'smart');  % id_fb
add_line(modelName, 'Park_Transform/2', 'Current_Controller/4', 'autorouting', 'smart');  % iq_fb

% 速度反馈 -> 速度控制器
add_line(modelName, 'BLDC_Motor/5', 'Speed_Controller/2', 'autorouting', 'smart');  % omega_m

% 连接示波器
add_line(modelName, 'Speed_Reference/1', 'Speed_Scope/1', 'autorouting', 'smart');
add_line(modelName, 'BLDC_Motor/5', 'Speed_Scope/2', 'autorouting', 'smart');
add_line(modelName, 'BLDC_Motor/5', 'speed_data/1', 'autorouting', 'smart');

% 电流示波器
add_line(modelName, 'Speed_Controller/2', 'Current_dq_Scope/1', 'autorouting', 'smart');
add_line(modelName, 'Park_Transform/1', 'Current_dq_Scope/2', 'autorouting', 'smart');
add_line(modelName, 'Speed_Controller/1', 'Current_dq_Scope/3', 'autorouting', 'smart');
add_line(modelName, 'Park_Transform/2', 'Current_dq_Scope/4', 'autorouting', 'smart');

% 转矩示波器
add_line(modelName, 'BLDC_Motor/6', 'Torque_Scope/1', 'autorouting', 'smart');

% 三相电流示波器
add_line(modelName, 'BLDC_Motor/1', 'Phase_Current_Scope/1', 'autorouting', 'smart');
add_line(modelName, 'BLDC_Motor/2', 'Phase_Current_Scope/2', 'autorouting', 'smart');
add_line(modelName, 'BLDC_Motor/3', 'Phase_Current_Scope/3', 'autorouting', 'smart');

fprintf('信号连接完成!\n');

%% ===================================================================
%  添加注释
%  ===================================================================
add_block('built-in/Note', [modelName '/Title_Note']);
set_param([modelName '/Title_Note'], ...
    'Position', [400 50 900 100], ...
    'Text', 'BLDC Motor FOC (Field-Oriented Control) System\n速度环 + 电流环双闭环矢量控制', ...
    'FontSize', '14', ...
    'FontWeight', 'bold');

%% 保存模型
save_system(modelName);
fprintf('\n============================================\n');
fprintf('  Simulink模型创建成功!\n');
fprintf('  模型名称: %s.slx\n', modelName);
fprintf('============================================\n');
fprintf('  使用说明:\n');
fprintf('  1. 先运行 motor_parameters.m 加载参数\n');
fprintf('  2. 打开模型: open_system(''%s'')\n', modelName);
fprintf('  3. 运行仿真: sim(''%s'')\n', modelName);
fprintf('============================================\n');
