function [i_alpha, i_beta] = clarke_transform(ia, ib, ic)
%% CLARKE_TRANSFORM Clarke变换 (abc -> alpha-beta)
%  将三相静止坐标系下的电流变换到两相静止坐标系
%
%  输入:
%    ia, ib, ic - 三相电流 [A]
%
%  输出:
%    i_alpha - alpha轴电流 [A]
%    i_beta  - beta轴电流 [A]
%
%  使用等幅值变换 (2/3 变换):
%    i_alpha = 2/3 * (ia - 1/2*ib - 1/2*ic)
%    i_beta  = 2/3 * (sqrt(3)/2*ib - sqrt(3)/2*ic)

    i_alpha = (2/3) * (ia - 0.5*ib - 0.5*ic);
    i_beta  = (2/3) * (sqrt(3)/2*ib - sqrt(3)/2*ic);
    
end
