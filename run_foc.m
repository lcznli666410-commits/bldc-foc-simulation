%% =========================================================================
%  BLDC FOC + SMO quick launcher
%  =========================================================================

clear; clc; close all;

fprintf('============================================\n');
fprintf('  BLDC FOC + SMO simulation system\n');
fprintf('============================================\n');
fprintf('  Select run mode:\n');
fprintf('    1 - MATLAB script simulation with SMO\n');
fprintf('    2 - Build modular Simulink model and run it\n');
fprintf('    3 - Build modular Simulink model only\n');
fprintf('    4 - Build/run modular Simulink model and export waveforms\n');
fprintf('============================================\n');

mode = input('Input selection (1/2/3/4): ');

switch mode
    case 1
        fprintf('\n>> Running MATLAB FOC simulation with SMO...\n\n');
        foc_simulation;

    case 2
        fprintf('\n>> Building Simulink model...\n\n');
        build_simulink_model;
        fprintf('\n>> Running Simulink model: %s\n', modelName);
        sim(modelName);
        fprintf('Simulation completed. Check scopes and foc_data.\n');

    case 3
        fprintf('\n>> Building Simulink model only...\n\n');
        build_simulink_model;
        fprintf('\nModel created: %s.slx\n', modelName);
        fprintf('Open with: open_system(''%s'')\n', modelName);

    case 4
        fprintf('\n>> Running modular Simulink validation with waveform export...\n\n');
        run_modular_foc_validation;

    otherwise
        fprintf('Invalid selection.\n');
end
