function [state, theta_hat, omega_e_hat, e_alpha_hat, e_beta_hat] = smo_observer( ...
    v_alpha, v_beta, i_alpha, i_beta, state, motor, smo, dt)
%% SMO_OBSERVER Sliding mode observer for surface PMSM/BLDC FOC.
%  Estimates alpha-beta back-EMF, rotor electrical angle, and electrical speed.

    err_alpha = state.i_alpha_hat - i_alpha;
    err_beta = state.i_beta_hat - i_beta;

    z_alpha = smo.Kslide * local_sat(err_alpha / smo.boundary);
    z_beta = smo.Kslide * local_sat(err_beta / smo.boundary);

    state.i_alpha_hat = state.i_alpha_hat + ...
        dt / motor.Ls * (v_alpha - motor.Rs * state.i_alpha_hat - z_alpha);
    state.i_beta_hat = state.i_beta_hat + ...
        dt / motor.Ls * (v_beta - motor.Rs * state.i_beta_hat - z_beta);

    emf_alpha = dt * smo.emf_lpf_cutoff / (1 + dt * smo.emf_lpf_cutoff);
    state.e_alpha_hat = state.e_alpha_hat + emf_alpha * (z_alpha - state.e_alpha_hat);
    state.e_beta_hat = state.e_beta_hat + emf_alpha * (z_beta - state.e_beta_hat);

    emf_mag = hypot(state.e_alpha_hat, state.e_beta_hat);
    if emf_mag >= smo.min_emf
        theta_lpf = atan2(-state.e_alpha_hat, state.e_beta_hat);
        phase_comp = atan2(state.omega_e_hat, smo.emf_lpf_cutoff);
        theta_raw = mod(theta_lpf + phase_comp, 2*pi);
        dtheta = atan2(sin(theta_raw - state.theta_hat), cos(theta_raw - state.theta_hat));
        omega_raw = dtheta / dt;

        speed_alpha = dt * smo.speed_lpf_cutoff / (1 + dt * smo.speed_lpf_cutoff);
        state.omega_e_hat = state.omega_e_hat + speed_alpha * (omega_raw - state.omega_e_hat);
        state.theta_hat = theta_raw;
    end

    theta_hat = state.theta_hat;
    omega_e_hat = state.omega_e_hat;
    e_alpha_hat = state.e_alpha_hat;
    e_beta_hat = state.e_beta_hat;
end

function y = local_sat(x)
    y = min(1, max(-1, x));
end
