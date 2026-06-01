function [Ta, Tb, Tc, sector] = svpwm(v_alpha, v_beta, Vdc)
%% SVPWM Center-aligned space-vector PWM.
%  Inputs:
%    v_alpha, v_beta - commanded alpha-beta voltage [V]
%    Vdc             - DC-link voltage [V]
%  Outputs:
%    Ta, Tb, Tc      - duty cycles [0, 1]
%    sector          - voltage vector sector [1, 6]

    va_ref = v_alpha;
    vb_ref = -0.5 * v_alpha + sqrt(3)/2 * v_beta;
    vc_ref = -0.5 * v_alpha - sqrt(3)/2 * v_beta;

    v_max = max([va_ref, vb_ref, vc_ref]);
    v_min = min([va_ref, vb_ref, vc_ref]);
    v_offset = -0.5 * (v_max + v_min);

    Ta = 0.5 + (va_ref + v_offset) / Vdc;
    Tb = 0.5 + (vb_ref + v_offset) / Vdc;
    Tc = 0.5 + (vc_ref + v_offset) / Vdc;

    Ta = max(0, min(1, Ta));
    Tb = max(0, min(1, Tb));
    Tc = max(0, min(1, Tc));

    angle = mod(atan2(v_beta, v_alpha), 2*pi);
    sector = floor(angle / (pi/3)) + 1;
    if sector < 1
        sector = 1;
    elseif sector > 6
        sector = 6;
    end
end
