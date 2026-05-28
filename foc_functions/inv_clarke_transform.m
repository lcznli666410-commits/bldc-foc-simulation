function [va, vb, vc] = inv_clarke_transform(v_alpha, v_beta)
%% INV_CLARKE_TRANSFORM 反Clarke变换 (alpha-beta -> abc)
%  将两相静止坐标系下的电压变换到三相静止坐标系
%
%  输入:
%    v_alpha - alpha轴电压 [V]
%    v_beta  - beta轴电压 [V]
%
%  输出:
%    va, vb, vc - 三相电压 [V]
%
%  使用等幅值反变换:
%    va = v_alpha
%    vb = -1/2*v_alpha + sqrt(3)/2*v_beta
%    vc = -1/2*v_alpha - sqrt(3)/2*v_beta

    va = v_alpha;
    vb = -0.5*v_alpha + sqrt(3)/2*v_beta;
    vc = -0.5*v_alpha - sqrt(3)/2*v_beta;
    
end
