%% =========================================================================
%  BLDC Motor Parameters Initialization
%  用于FOC控制仿真的电机参数配置
%  =========================================================================

%% 电机电气参数
motor.Rs = 0.5;          % 定子电阻 [Ohm]
motor.Ld = 0.8e-3;       % d轴电感 [H]
motor.Lq = 0.8e-3;       % q轴电感 [H] (表贴式永磁电机 Ld = Lq)
motor.Ls = motor.Ld;     % 定子电感 (对称电机)
motor.Ke = 0.0175;       % 反电动势常数 [V/(rad/s)]  (线反电动势常数)
motor.Kt = 0.0175;       % 转矩常数 [Nm/A]
motor.flux = 0.0175;     % 永磁体磁链 [Wb]
motor.pole_pairs = 4;    % 极对数

%% 电机机械参数
motor.J = 0.0001;        % 转动惯量 [kg*m^2]
motor.B = 0.0001;        % 粘滞摩擦系数 [Nm/(rad/s)]
motor.TL = 0.0;          % 负载转矩 [Nm] (初始值)

%% 电源参数
motor.Vdc = 24;          % 直流母线电压 [V]
motor.Vmax = motor.Vdc / sqrt(3);  % 最大相电压幅值

%% 额定参数
motor.rated_speed = 3000;         % 额定转速 [rpm]
motor.rated_current = 10;         % 额定电流 [A]
motor.rated_torque = 0.175;       % 额定转矩 [Nm]
motor.max_speed = 5000;           % 最大转速 [rpm]
motor.max_current = 15;           % 最大电流 [A]

%% PWM参数
pwm.freq = 20000;        % PWM频率 [Hz]
pwm.Ts = 1/pwm.freq;     % PWM周期 [s]
pwm.deadtime = 1e-6;     % 死区时间 [s]

%% 控制参数 - 电流环PI控制器
% d轴电流控制器
pid_id.Kp = 5.0;         % 比例增益
pid_id.Ki = 1000;        % 积分增益
pid_id.Kd = 0;           % 微分增益
pid_id.max = motor.Vdc/2;  % 输出限幅
pid_id.min = -motor.Vdc/2;

% q轴电流控制器
pid_iq.Kp = 5.0;         % 比例增益
pid_iq.Ki = 1000;        % 积分增益
pid_iq.Kd = 0;           % 微分增益
pid_iq.max = motor.Vdc/2;  % 输出限幅
pid_iq.min = -motor.Vdc/2;

%% 控制参数 - 速度环PI控制器
pid_speed.Kp = 0.5;      % 比例增益
pid_speed.Ki = 10;       % 积分增益
pid_speed.Kd = 0;        % 微分增益
pid_speed.max = motor.max_current;   % 输出限幅 (iq参考值)
pid_speed.min = -motor.max_current;

%% SMO滑膜观测器参数
smo.Kslide = 30;                % 滑膜增益 [V]
smo.boundary = 0.2;             % 饱和边界层 [A]
smo.emf_lpf_cutoff = 2000;      % 反电动势低通截止角频率 [rad/s]
smo.speed_lpf_cutoff = 500;     % 估计速度低通截止角频率 [rad/s]
smo.min_emf = 0.05;             % 低速反电动势门限 [V]
smo.enable_time = 0.02;         % 允许使用SMO反馈的时间 [s]

%% 仿真参数
sim_params.Ts = 1e-6;            % 仿真步长 [s]
sim_params.Tend = 0.5;           % 仿真结束时间 [s]
sim_params.speed_ref = 1000;     % 速度参考值 [rpm]
sim_params.load_time = 0.3;      % 加载负载时间 [s]
sim_params.load_torque = 0.1;    % 负载转矩 [Nm]
sim_params.use_smo_feedback = true;   % true时使用SMO估计角度/速度闭环

%% 显示参数信息
fprintf('============================================\n');
fprintf('  BLDC电机FOC控制 - 参数初始化完成\n');
fprintf('============================================\n');
fprintf('  极对数: %d\n', motor.pole_pairs);
fprintf('  定子电阻: %.3f Ohm\n', motor.Rs);
fprintf('  d轴电感: %.3f mH\n', motor.Ld*1000);
fprintf('  q轴电感: %.3f mH\n', motor.Lq*1000);
fprintf('  永磁磁链: %.4f Wb\n', motor.flux);
fprintf('  转动惯量: %.6f kg*m^2\n', motor.J);
fprintf('  直流母线电压: %.1f V\n', motor.Vdc);
fprintf('  PWM频率: %.0f Hz\n', pwm.freq);
fprintf('  额定转速: %.0f rpm\n', motor.rated_speed);
fprintf('============================================\n');
