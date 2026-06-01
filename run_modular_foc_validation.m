%% =========================================================================
%  BLDC FOC + SMO modular Simulink validation
%  =========================================================================
%  Builds the modular Simulink model, runs it, saves validation data, and
%  exports a waveform image without requiring the Simulink GUI.
%  =========================================================================

clear; clc; close all;

build_simulink_model;

simOut = sim(modelName);
if exist('foc_data', 'var')
    data = foc_data;
else
    data = simOut.get('foc_data');
end

values = squeeze(data.signals.values);
if size(values, 1) ~= numel(data.time)
    values = values.';
end

t = data.time;
speed_ref = values(:, 1) * 60 / (2*pi);
speed_fb = values(:, 2) * 60 / (2*pi);
speed_smo = values(:, 3) * 60 / (2*pi);
id = values(:, 4);
iq_ref = values(:, 5);
iq = values(:, 6);
ia = values(:, 7);
ib = values(:, 8);
ic = values(:, 9);
Te = values(:, 10);
TL = values(:, 11);
theta_e = values(:, 12);
theta_smo = values(:, 13);
theta_err_deg = values(:, 14) * 180 / pi;
duty_a = values(:, 15);
duty_b = values(:, 16);
duty_c = values(:, 17);

steadyStart = max(1, round(numel(t) * 0.7));
steady = steadyStart:numel(t);

summary.final_speed_rpm = speed_fb(end);
summary.speed_error_rpm = mean(abs(speed_fb(steady) - speed_ref(steady)));
summary.id_a = mean(id(steady));
summary.iq_a = mean(iq(steady));
summary.phase_current_peak_a = max(max(abs([ia(steady), ib(steady), ic(steady)])));
summary.smo_speed_error_rpm = mean(abs(speed_smo(steady) - speed_fb(steady)));
summary.smo_angle_error_deg = mean(abs(theta_err_deg(steady)));

save('build_model_validation.mat', 'modelName', 'data', 'summary');

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1500 1000]);
tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(t, speed_ref, '--', 'LineWidth', 1.1); hold on;
plot(t, speed_fb, 'LineWidth', 1.4);
plot(t, speed_smo, ':', 'LineWidth', 1.2);
grid on; title('Speed Reference / Motor / SMO');
xlabel('Time (s)'); ylabel('Speed (rpm)');
legend('ref', 'motor', 'SMO', 'Location', 'southeast');

nexttile;
plot(t, ia, 'LineWidth', 1.0); hold on;
plot(t, ib, 'LineWidth', 1.0);
plot(t, ic, 'LineWidth', 1.0);
grid on; title('Three Phase Currents');
xlabel('Time (s)'); ylabel('Current (A)');
legend('ia', 'ib', 'ic', 'Location', 'best');

nexttile;
plot(t, id, 'LineWidth', 1.1); hold on;
plot(t, iq_ref, '--', 'LineWidth', 1.1);
plot(t, iq, 'LineWidth', 1.1);
grid on; title('DQ Current Loop');
xlabel('Time (s)'); ylabel('Current (A)');
legend('id', 'iq ref', 'iq', 'Location', 'best');

nexttile;
plot(t, theta_e, 'LineWidth', 1.0); hold on;
plot(t, theta_smo, '--', 'LineWidth', 1.0);
grid on; title('Electrical Angle: Motor vs SMO');
xlabel('Time (s)'); ylabel('Angle (rad)');
legend('motor', 'SMO', 'Location', 'best');

nexttile;
plot(t, theta_err_deg, 'LineWidth', 1.0);
grid on; title('SMO Angle Error');
xlabel('Time (s)'); ylabel('Error (deg)');

nexttile;
plot(t, duty_a, 'LineWidth', 1.0); hold on;
plot(t, duty_b, 'LineWidth', 1.0);
plot(t, duty_c, 'LineWidth', 1.0);
plot(t, Te, 'k', 'LineWidth', 1.0);
plot(t, TL, 'k--', 'LineWidth', 1.0);
grid on; title('SVPWM Duty + Torque');
xlabel('Time (s)'); ylabel('Duty / Nm');
legend('Ta', 'Tb', 'Tc', 'Te', 'TL', 'Location', 'best');

exportgraphics(fig, 'modular_foc_waveforms.png', 'Resolution', 180);
close(fig);

fprintf('\n============================================\n');
fprintf('  Modular FOC + SMO validation completed\n');
fprintf('============================================\n');
fprintf('  Model: %s.slx\n', modelName);
fprintf('  Data:  build_model_validation.mat\n');
fprintf('  Plot:  modular_foc_waveforms.png\n');
fprintf('  Final speed: %.3f rpm\n', summary.final_speed_rpm);
fprintf('  Speed error: %.3f rpm\n', summary.speed_error_rpm);
fprintf('  SMO speed error: %.3f rpm\n', summary.smo_speed_error_rpm);
fprintf('  SMO angle error: %.3f deg\n', summary.smo_angle_error_deg);
fprintf('============================================\n');
