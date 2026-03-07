clear;
clc;

%% § 符号定义

% 物理常数
syms g real

% 物理参数
syms mb ml mw real
syms Ib Ill Ilr Iw Iphi real 
syms db thetab0 real
syms dll dlr thetall0 thetalr0 real
syms Rb Rw real

% 广义坐标
syms x phi thetall thetalr thetab real
% 广义速度
syms dx dphi dthetall dthetalr dthetab real
% 广义加速度
syms ddx ddphi ddthetall ddthetalr ddthetab real
% 腿长
syms ll lr real

% 控制力矩
syms Tll Tlr Twl Twr real

%% § 运动学约束

a_l_h =  ll*cos(thetall)*ddthetall-ll*sin(thetall)*dthetall^2 ...
       + lr*cos(thetalr)*ddthetalr-lr*sin(thetalr)*dthetalr^2;

a_ll_v = -ll*sin(thetall)*ddthetall-ll*cos(thetall)*dthetall^2;
a_lr_v = -lr*sin(thetalr)*ddthetalr-lr*cos(thetalr)*dthetalr^2;
    
ddthetawl = (ddx-Rb*ddphi+1/2*a_l_h)/Rw;
ddthetawr = (ddx+Rb*ddphi+1/2*a_l_h)/Rw;


%% § 动力学方程

% 水平动量方程
eqn1 = (Twl-Iw*ddthetawl+Twr-Iw*ddthetawr) - ((1/2*mb+ml+mw)*Rw*(ddthetawl+ddthetawr)+(1/2*mb+ml)*a_l_h)*Rw;

% 偏航转动方程
eqn2 = (Iphi*ddphi) - ((Twr-Iw*ddthetawr)-(Twl-Iw*ddthetawl))*Rb/Rw;

% 左腿转动方程
eqn3 = Tll-Twl-ml*g*dll*sin(thetall+thetall0) ...
       -((Twl-Iw*ddthetawl)/Rw-ml*Rw*ddthetawl)*ll*cos(thetall) ...
       +(1/2*mb+ml)*(g+(a_ll_v+a_lr_v)/2)*ll*sin(thetall) ...
       - Ill*ddthetall;

% 右腿转动方程
eqn4 = Tlr-Twr-ml*g*dlr*sin(thetalr+thetalr0) ...
       -((Twr-Iw*ddthetawr)/Rw-ml*Rw*ddthetawr)*lr*cos(thetalr) ...
       +(1/2*mb+ml)*(g+(a_ll_v+a_lr_v)/2)*lr*sin(thetalr) ...
       - Ilr*ddthetalr;

% 机体转动方程
eqn5 = (-Tll-Tlr+mb*g*db*cos(thetab+thetab0)) - (Ib*ddthetab);


%% § 平衡点解算 

thetall_eq = atan2(ml*dll*sin(thetall0), (1/2*mb+ml)*ll-ml*dll*cos(thetall0));
thetalr_eq = atan2(ml*dlr*sin(thetalr0), (1/2*mb+ml)*lr-ml*dlr*cos(thetalr0));

%% § 物理参数

% ==================== 物理常数 ====================
g_val = 9.78;              % 重力加速度 (m/s^2)

% ==================== 几何参数 ====================
Rw_val = 0.0752;              % 轮子半径 (m)
Rb_val = 0.362/2;             % 轮距/2 (m)

% ==================== 机体参数 ====================
mb_val = 13.845198;             % 机体质量 (kg)
Ib_val = 0.462592306;           % 机体俯仰转动惯量 (kg·m²)
dx_val = 0.016878;
dy_val = 0.126476;
db_val = 0.1275972;             % 机体质心到俯仰轴距离 (m)
Iphi_val = 0.314373184;         % 整体yaw轴转动惯量 (kg·m²)
thetab0_val = pi/2;%1.438131894;      % 质心偏移角度 (rad)

% ==================== 轮子参数 ====================
mw_val = 0.21;                % 轮质量 (kg)
Iw_val = 0.00038332661;       % 轮转动惯量 (kg·m²)

% ==================== 腿部参数 ====================
ml_val = 1.62;              % 腿质量 (kg)

params_subs =  {
    g, g_val;
    Rb, Rb_val;
    Rw, Rw_val;
    mb, mb_val;
    Ib, Ib_val;
    db, db_val;
    Iphi, Iphi_val;
    thetab0, thetab0_val;
    mw, mw_val;
    Iw, Iw_val;
    ml, ml_val;
};

thetall_eq = simplify(subs(thetall_eq, params_subs(:,1), params_subs(:,2)));
thetalr_eq = simplify(subs(thetalr_eq, params_subs(:,1), params_subs(:,2)));
eqn1 = simplify(subs(eqn1, params_subs(:,1), params_subs(:,2)));
eqn2 = simplify(subs(eqn2, params_subs(:,1), params_subs(:,2)));
eqn3 = simplify(subs(eqn3, params_subs(:,1), params_subs(:,2)));
eqn4 = simplify(subs(eqn4, params_subs(:,1), params_subs(:,2)));
eqn5 = simplify(subs(eqn5, params_subs(:,1), params_subs(:,2)));

u = [Twl; Twr; Tll; Tlr];
q = [x; phi; thetall; thetalr; thetab];
dq = [dx; dphi; dthetall; dthetalr; dthetab];
ddq = [ddx; ddphi; ddthetall; ddthetalr; ddthetab];
eqns_vec = [eqn1; eqn2; eqn3; eqn4; eqn5];

%% § 提取偏导数矩阵

% M*ddq + D*dq + K*q + H*u = 0
fprintf('Solving M*ddq + D*dq + K*q + H*u = 0 ...\n');
% 质量/惯量矩阵 M = dF / d(ddq)
M_sym = jacobian(eqns_vec, ddq);

% 阻尼/科里奥利矩阵 D = dF / d(dq)
D_sym = jacobian(eqns_vec, dq);

% 刚度/重力矩阵 K = dF / d(q)
K_sym = jacobian(eqns_vec, q);

% 输入矩阵 H = dF / d(u)
H_sym = jacobian(eqns_vec, u);

% 代入物理参数
M_param = simplify(subs(M_sym, params_subs(:,1), params_subs(:,2)));
D_param = simplify(subs(D_sym, params_subs(:,1), params_subs(:,2)));
K_param = simplify(subs(K_sym, params_subs(:,1), params_subs(:,2)));
H_param = simplify(subs(H_sym, params_subs(:,1), params_subs(:,2)));

%% § 在平衡点处计算矩阵的值

eq_subs = {
    thetall, thetall_eq;
    thetalr, thetalr_eq;
    thetab, pi/2-thetab0_val;
    dthetall, 0; dthetalr, 0; dthetab, 0;
    phi, 0; dphi, 0;
    x, 0; dx, 0;
    ddx, 0; ddphi, 0; ddthetall, 0; ddthetalr, 0; ddthetab, 0;
    Twl, 0; Twr, 0; Tll, 0; Tlr, 0;
};

M_eq = simplify(subs(M_param, eq_subs(:,1), eq_subs(:,2)));
D_eq = simplify(subs(D_param, eq_subs(:,1), eq_subs(:,2)));
K_eq = simplify(subs(K_param, eq_subs(:,1), eq_subs(:,2)));
H_eq = simplify(subs(H_param, eq_subs(:,1), eq_subs(:,2)));

%% § 构建线性化状态空间矩阵 A 和 B

n = 10;         % [x, dx, phi, dphi, thetall, dthetall, thetalr, dthetalr, thetab, dthetab]^T
m_ctrl = 4;     % [Twl, Twr, Tll, Tlr]^T

A_num = sym(zeros(n, n));
B_num = sym(zeros(n, m_ctrl));

% 运动学关系: 导数关系
A_num(1,2) = 1;   % dx/dt = dx
A_num(3,4) = 1;   % dphi/dt = dphi
A_num(5,6) = 1;   % dthetall/dt = dthetall
A_num(7,8) = 1;   % dthetalr/dt = dthetalr
A_num(9,10) = 1;  % dthetab/dt = dthetab

% 计算 M 的逆
M_inv = inv(M_eq);

% 根据公式: ddq = -M^{-1}*K * q - M^{-1}*D * dq - M^{-1}*H * u
A_dyn_K = -M_inv * K_eq;  % 对应位置 q 的系数
A_dyn_D = -M_inv * D_eq;  % 对应速度 dq 的系数
B_dyn   = -M_inv * H_eq;  % 对应输入 u 的系数

% 将动力学部分填入 A 矩阵 (偶数行为加速度方程)
% 映射位置状态项 (q: 列 1, 3, 5, 7, 9)
A_num(2:2:10, 1:2:9) = A_dyn_K;

% 映射速度状态项 (dq: 列 2, 4, 6, 8, 10)
A_num(2:2:10, 2:2:10) = A_dyn_D;

% 填入 B 矩阵
B_num(2:2:10, :) = B_dyn;

fprintf('Generating matrices.m ...\n');
matlabFunction(A_num, B_num, thetall_eq, thetalr_eq, ...
    'File', 'matrices', ...
    'Vars', {ll, lr, thetall0, thetalr0, dll, dlr, Ill, Ilr}, ...
    'Outputs', {'A', 'B', 'thetall_eq', 'thetalr_eq'});
fprintf('✓ Done! \n');
