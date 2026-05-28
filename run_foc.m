%% =========================================================================
%  BLDC FOC控制 - 快速启动脚本
%  =========================================================================
%  此脚本提供两种仿真模式:
%  1. 纯MATLAB脚本仿真 (不需要Simulink)
%  2. Simulink模型仿真
%  =========================================================================

clear; clc; close all;

fprintf('============================================\n');
fprintf('  BLDC电机 FOC矢量控制仿真系统\n');
fprintf('  ==========================================\n');
fprintf('  请选择运行模式:\n');
fprintf('    1 - 纯MATLAB脚本仿真 (推荐，无需Simulink)\n');
fprintf('    2 - 构建并运行Simulink模型\n');
fprintf('    3 - 仅构建Simulink模型 (不运行)\n');
fprintf('============================================\n');

mode = input('请输入选择 (1/2/3): ');

switch mode
    case 1
        fprintf('\n>> 启动纯MATLAB FOC仿真...\n\n');
        foc_simulation;
        
    case 2
        fprintf('\n>> 构建Simulink模型并运行仿真...\n\n');
        build_simulink_model;
        fprintf('\n>> 运行仿真...\n');
        sim('BLDC_FOC_Model');
        fprintf('仿真完成！请查看Scope窗口\n');
        
    case 3
        fprintf('\n>> 仅构建Simulink模型...\n\n');
        build_simulink_model;
        fprintf('\n模型已创建，可通过以下命令打开:\n');
        fprintf('  open_system(''BLDC_FOC_Model'')\n');
        
    otherwise
        fprintf('无效选择!\n');
end
