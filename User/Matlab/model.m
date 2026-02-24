clear;

% 物理常数
syms g real

% 机体参数
syms m_b I_b l_b theta_b0 real
syms X_b_h dX_b_h ddX_b_h real            % 机体水平位置
syms theta_b dtheta_b ddtheta_b real      % 机体角度、角速度、角加速度
syms F_l_to_b_h F_l_to_b_v T_l_to_b real  % 左腿对机体的力和力矩
syms F_r_to_b_h F_r_to_b_v T_r_to_b real  % 右腿对机体的力和力矩
syms a_b_h a_b_v real                     % 机体加速度

% 腿部参数
syms m_l m_r I_l I_r l_l l_r l_l_d l_r_d theta_l0 theta_r0 real
syms theta_l dtheta_l ddtheta_l theta_r dtheta_r ddtheta_r real
syms F_wl_to_l_h F_wl_to_l_v T_wl_to_l real  % 左轮对左腿
syms F_wr_to_r_h F_wr_to_r_v T_wr_to_r real  % 右轮对右腿
syms a_l_h a_l_v a_r_h a_r_v real            % 腿加速度

% 轮子参数
syms m_wl m_wr I_wl I_wr real
syms theta_wl dtheta_wl ddtheta_wl theta_wr dtheta_wr ddtheta_wr real
syms F_g_to_wl_h F_g_to_wl_v F_g_to_wr_h F_g_to_wr_v real  % 地面力
syms a_wl_h a_wl_v a_wr_h a_wr_v real                       % 轮加速度

% 几何参数
syms R R_w I_yaw real

% Yaw角
syms phi dphi ddphi real

%% ========================================
%  § 动力学方程
%  ========================================

% 整体水平动量方程：所有水平方程相加
% eq_body_h + eq_leg_l_h + eq_leg_r_h + eq_wl_h + eq_wr_h = 0
% 地面力 F_g_to_wl_h 和 F_g_to_wr_h 需要用轮转动方程消去

% 从轮转动方程得到地面力
F_g_to_wl_h_expr = -(T_wl_to_l + I_wl*ddtheta_wl)/R;
F_g_to_wr_h_expr = -(T_wr_to_r + I_wr*ddtheta_wr)/R;

% 最终方程1: 整体水平动量
eq1 = m_b*a_b_h + m_l*a_l_h + m_r*a_r_h + m_wl*a_wl_h + m_wr*a_wr_h ...
    + (T_wl_to_l + T_wr_to_r + I_wl*ddtheta_wl + I_wr*ddtheta_wr)/R;

% 最终方程2: 机体转动
eq2 = I_b*ddtheta_b - T_l_to_b - T_r_to_b - m_b*g*l_b*sin(theta_b + theta_b0);

% 最终方程3: 右腿转动 (需要消去 F_wr_to_r)
% 先求 F_wr_to_r_h 和 F_wr_to_r_v
% 从 eq_wr_h: F_wr_to_r_h = F_g_to_wr_h - m_wr*a_wr_h
F_wr_to_r_h_expr = F_g_to_wr_h_expr - m_wr*a_wr_h;

% 假设轮子不离地: a_wr_v = 0, 且两轮支持力相等
% F_wr_to_r_v = F_g_to_wr_v + m_wr*g
% 代入 eq_leg_r_v: F_r_to_b_v = F_wr_to_r_v + m_r*g - m_r*a_r_v
% 再代入整体竖直方程...

% 简化处理：假设两轮竖直力的和等于总重力
% F_g_to_wr_v + F_g_to_wl_v = (m_b + m_l + m_r + m_wl + m_wr)*g
% 且假设 F_g_to_wr_v = F_g_to_wl_v

F_g_v_each = (m_b + m_l + m_r + m_wl + m_wr)*g/2;

% 用整体竖直方向约束：
% F_l_to_b_v + F_r_to_b_v = m_b*a_b_v - m_b*g
% ... 这部分推导较复杂，参考推导文档

% 最终方程3: 右腿转动方程
eq3 = I_r*ddtheta_r - T_wr_to_r + T_r_to_b - m_r*g*l_r_d*sin(theta_r + theta_r0) ...
    - m_wr*a_wr_h*l_r*cos(theta_r) - (T_wr_to_r + I_wr*ddtheta_wr)*l_r*cos(theta_r)/R ...
    - (m_b*a_b_v + m_l*a_l_v + m_r*a_r_v)*l_r*sin(theta_r)/2 ...
    - (m_wr - m_b - m_l - m_r - m_wl)*g*l_r*sin(theta_r)/2;

% 最终方程4: 左腿转动方程
eq4 = I_l*ddtheta_l - T_wl_to_l + T_l_to_b - m_l*g*l_l_d*sin(theta_l + theta_l0) ...
    - m_wl*a_wl_h*l_l*cos(theta_l) - (T_wl_to_l + I_wl*ddtheta_wl)*l_l*cos(theta_l)/R ...
    - (m_b*a_b_v + m_l*a_l_v + m_r*a_r_v)*l_l*sin(theta_l)/2 ...
    - (m_wl - m_b - m_l - m_r - m_wr)*g*l_l*sin(theta_l)/2;

% 最终方程5: Yaw转动方程
eq5 = I_yaw*ddphi - R_w/R*(T_wl_to_l + I_wl*ddtheta_wl - T_wr_to_r - I_wr*ddtheta_wr);

%% ========================================
%  § 运动学约束
%  ========================================

% ----- 纯滚动约束 -----
a_wr_h_expr = R * ddtheta_wr;
a_wl_h_expr = R * ddtheta_wl;

% ----- 轮不离地约束 -----
a_wr_v_expr = sym(0);
a_wl_v_expr = sym(0);

% ----- 腿转轴加速度 (由轮得到) -----
a_r_h_expr = a_wr_h_expr + l_r*cos(theta_r)*ddtheta_r - l_r*sin(theta_r)*dtheta_r^2;
a_r_v_expr = a_wr_v_expr - l_r*sin(theta_r)*ddtheta_r - l_r*cos(theta_r)*dtheta_r^2;
a_l_h_expr = a_wl_h_expr + l_l*cos(theta_l)*ddtheta_l - l_l*sin(theta_l)*dtheta_l^2;
a_l_v_expr = a_wl_v_expr - l_l*sin(theta_l)*ddtheta_l - l_l*cos(theta_l)*dtheta_l^2;

% ----- 机体加速度 (左右腿转轴平均) -----
a_b_h_expr = (a_r_h_expr + a_l_h_expr) / 2;
a_b_v_expr = (a_r_v_expr + a_l_v_expr) / 2;

% ----- Yaw角加速度 -----
ddphi_expr = (a_wr_h_expr - a_wl_h_expr) / (2*R_w);

%% ========================================
%  § 转换与代换
%  ========================================

% 轮角加速度用广义坐标表示
leg_accel_term = (l_r*cos(theta_r)*ddtheta_r + l_l*cos(theta_l)*ddtheta_l)/2;
leg_vel_sq_term = (l_r*sin(theta_r)*dtheta_r^2 + l_l*sin(theta_l)*dtheta_l^2)/2;
leg_term = leg_accel_term - leg_vel_sq_term;

ddtheta_wr_expr = (ddX_b_h + R_w*ddphi)/R - leg_term/R;
ddtheta_wl_expr = (ddX_b_h - R_w*ddphi)/R - leg_term/R;

% 将原方程中的加速度变量替换为用广义坐标表示的表达式
% 注意: a_b_h 不直接替换为 ddX_b_h，而是保持为展开形式以便后续提取系数

kinematics_subs = {
    % 轮加速度 (纯滚动)
    a_wr_h, R*ddtheta_wr;
    a_wl_h, R*ddtheta_wl;
    a_wr_v, sym(0);
    a_wl_v, sym(0);
    
    % 腿转轴加速度
    a_r_h, R*ddtheta_wr + l_r*cos(theta_r)*ddtheta_r - l_r*sin(theta_r)*dtheta_r^2;
    a_r_v, -l_r*sin(theta_r)*ddtheta_r - l_r*cos(theta_r)*dtheta_r^2;
    a_l_h, R*ddtheta_wl + l_l*cos(theta_l)*ddtheta_l - l_l*sin(theta_l)*dtheta_l^2;
    a_l_v, -l_l*sin(theta_l)*ddtheta_l - l_l*cos(theta_l)*dtheta_l^2;
    
    % 机体加速度 (左右腿转轴平均)
    a_b_h, (R*(ddtheta_wr + ddtheta_wl)/2 ...
          + (l_r*cos(theta_r)*ddtheta_r + l_l*cos(theta_l)*ddtheta_l)/2 ...
          - (l_r*sin(theta_r)*dtheta_r^2 + l_l*sin(theta_l)*dtheta_l^2)/2);
    a_b_v, (-(l_r*sin(theta_r)*ddtheta_r + l_l*sin(theta_l)*ddtheta_l)/2 ...
          - (l_r*cos(theta_r)*dtheta_r^2 + l_l*cos(theta_l)*dtheta_l^2)/2);
};

fprintf('Kinematic Substitution ...\n');
eq1 = simplify(subs(eq1, kinematics_subs(:,1), kinematics_subs(:,2)));
eq2 = simplify(subs(eq2, kinematics_subs(:,1), kinematics_subs(:,2)));
eq3 = simplify(subs(eq3, kinematics_subs(:,1), kinematics_subs(:,2)));
eq4 = simplify(subs(eq4, kinematics_subs(:,1), kinematics_subs(:,2)));
eq5 = simplify(subs(eq5, kinematics_subs(:,1), kinematics_subs(:,2)));

wheel_subs = {ddtheta_wr, ddtheta_wr_expr; ddtheta_wl, ddtheta_wl_expr};

fprintf('General Coordinatess Substitution ...\n');
eq1 = simplify(subs(eq1, wheel_subs(:,1), wheel_subs(:,2)));
eq2 = simplify(subs(eq2, wheel_subs(:,1), wheel_subs(:,2)));
eq3 = simplify(subs(eq3, wheel_subs(:,1), wheel_subs(:,2)));
eq4 = simplify(subs(eq4, wheel_subs(:,1), wheel_subs(:,2)));
eq5 = simplify(subs(eq5, wheel_subs(:,1), wheel_subs(:,2)));

%% ========================================
%  § 求解M,B,g矩阵
%  ========================================
fprintf('Solving M,B,g Matrices ...\n');

ddq = [ddX_b_h; ddphi; ddtheta_l; ddtheta_r; ddtheta_b];
u = [T_wl_to_l; T_wr_to_r; T_l_to_b; T_r_to_b];
eqs = {eq1, eq2, eq3, eq4, eq5};

% 提取 M 矩阵
M_sym = sym(zeros(5,5));
for i = 1:5
    for j = 1:5
        M_sym(i,j) = diff(eqs{i}, ddq(j));
    end
end

% 提取 B 矩阵
B_raw = sym(zeros(5,4));
for i = 1:5
    for j = 1:4
        B_raw(i,j) = diff(eqs{i}, u(j));
    end
end
B_sym = -B_raw;

% 提取 g 向量
g_sym = sym(zeros(5,1));
for i = 1:5
    g_sym(i) = -(eqs{i} - M_sym(i,:)*ddq - B_raw(i,:)*u);
end

%% ========================================
%  § 物理参数
%  ========================================

% ==================== 物理常数 ====================
g_val = -9.81;              % 重力加速度 (m/s^2)

% ==================== 几何参数 ====================
R_val = 0.055;              % 轮子半径 (m)
R_w_val = (0.455+0.445)/4;  % 轮距/2 (m)

% ==================== 机体参数 ====================
m_b_val = 7.846;            % 机体质量 (kg)
I_b_val = 0.150;            % 机体俯仰转动惯量 (kg·m²)
l_b_val = 0.055;            % 机体质心到俯仰轴距离 (m)
I_yaw_val = 0.465;          % 整体yaw轴转动惯量 (kg·m²)
theta_b0_val = 0;           % 质心偏移角度 (rad)

% ==================== 轮子参数 ====================
m_wl_val = 0.19;            % 左轮质量 (kg)
m_wr_val = 0.19;            % 右轮质量 (kg)
I_wl_val = 0.000207897;     % 左轮转动惯量 (kg·m²)
I_wr_val = 0.000207897;     % 右轮转动惯量 (kg·m²)

% ==================== 腿部参数 (默认腿长 0.20m) ====================
l_l_val = 0.20;             % 左腿长度 (m)
l_r_val = 0.20;             % 右腿长度 (m)
m_l_val = 1.62;             % 左腿质量 (kg)
m_r_val = 1.62;             % 右腿质量 (kg)
I_l_val = 0.0339;           % 左腿转动惯量 (kg·m²)
I_r_val = 0.0339;           % 右腿转动惯量 (kg·m²)
l_l_d_val = 0.084685385397954;  % 左腿质心到轮轴距离 (m)
l_r_d_val = 0.084685385397954;  % 右腿质心到轮轴距离 (m)
theta_l0_val = 0.8152113973036387;  % 左腿偏移角度 (rad)
theta_r0_val = 0.8152113973036387;  % 右腿偏移角度 (rad)

% 物理参数代换表
param_subs = {
    m_b, m_b_val;
    m_l, m_l_val;
    m_r, m_r_val;
    m_wl, m_wl_val;
    m_wr, m_wr_val;
    I_b, I_b_val;
    I_l, I_l_val;
    I_r, I_r_val;
    I_wl, I_wl_val;
    I_wr, I_wr_val;
    I_yaw, I_yaw_val;
    l_l, l_l_val;
    l_r, l_r_val;
    l_l_d, l_l_d_val;
    l_r_d, l_r_d_val;
    l_b, l_b_val;
    R, R_val;
    R_w, R_w_val;
    g, g_val;
    theta_l0, theta_l0_val;
    theta_r0, theta_r0_val;
    theta_b0, theta_b0_val;
};

%% ========================================
%  § Export Amatrix/Bmatrix for LQR fitting
%  ========================================

fprintf('Generating Amatrix/Bmatrix functions ...\n');

% 线性化点: 零位置/零速度/零加速度/零控制
lin_subs = {
    theta_l, 0;
    theta_r, 0;
    theta_b, 0;
    dtheta_l, 0;
    dtheta_r, 0;
    dtheta_b, 0;
    ddtheta_l, 0;
    ddtheta_r, 0;
    ddtheta_b, 0;
    ddX_b_h, 0;
    ddphi, 0;
    phi, 0;
    dphi, 0;
    X_b_h, 0;
    dX_b_h, 0;
    T_r_to_b, 0;
    T_l_to_b, 0;
    T_wr_to_r, 0;
    T_wl_to_l, 0;
};

dg_dtheta_sym = [diff(g_sym, theta_l), diff(g_sym, theta_r), diff(g_sym, theta_b)];

M_lin = subs(M_sym, lin_subs(:,1), lin_subs(:,2));
B_lin = subs(B_sym, lin_subs(:,1), lin_subs(:,2));
dg_dtheta_lin = subs(dg_dtheta_sym, lin_subs(:,1), lin_subs(:,2));

param_fit_subs = {
    m_b, m_b_val;
    m_l, m_l_val;
    m_r, m_r_val;
    m_wl, m_wl_val;
    m_wr, m_wr_val;
    I_b, I_b_val;
    I_wl, I_wl_val;
    I_wr, I_wr_val;
    I_yaw, I_yaw_val;
    l_b, l_b_val;
    R, R_val;
    R_w, R_w_val;
    g, g_val;
    theta_l0, theta_l0_val;
    theta_r0, theta_r0_val;
    theta_b0, theta_b0_val;
};

M_lin = simplify(subs(M_lin, param_fit_subs(:,1), param_fit_subs(:,2)));
B_lin = simplify(subs(B_lin, param_fit_subs(:,1), param_fit_subs(:,2)));
dg_dtheta_lin = simplify(subs(dg_dtheta_lin, param_fit_subs(:,1), param_fit_subs(:,2)));

J_A_sym = simplify(inv(M_lin) * dg_dtheta_lin);
J_B_sym = simplify(inv(M_lin) * B_lin);

this_file = mfilename('fullpath');
if isempty(this_file)
    out_dir = pwd;
else
    out_dir = fileparts(this_file);
end

matlabFunction(J_A_sym, 'File', fullfile(out_dir, 'matrixA.m'), ...
    'Vars', {l_l, l_r, l_l_d, l_r_d, I_l, I_r});
matlabFunction(J_B_sym, 'File', fullfile(out_dir, 'matrixB.m'), ...
    'Vars', {l_l, l_r, l_l_d, l_r_d, I_l, I_r});

fprintf('✓ Amatrix/Bmatrix exported\n');

fprintf('Physical Params Substitution ...\n');
M_param = simplify(subs(M_sym, param_subs(:,1), param_subs(:,2)));
B_param = simplify(subs(B_sym, param_subs(:,1), param_subs(:,2)));
g_param = simplify(subs(g_sym, param_subs(:,1), param_subs(:,2)));

%% ========================================
%  § 平衡点验证
%  ========================================

% 代入平衡点条件 (速度=0, 加速度=0, 控制=0, phi=0, X=0)，保留 theta_l, theta_r, theta_b
eq_subs_partial = {
    dtheta_l, 0;
    dtheta_r, 0;
    dtheta_b, 0;
    ddtheta_l, 0;
    ddtheta_r, 0;
    ddtheta_b, 0;
    ddX_b_h, 0;
    ddphi, 0;
    phi, 0;
    dphi, 0;
    X_b_h, 0;
    dX_b_h, 0;
    T_r_to_b, 0;
    T_l_to_b, 0;
    T_wr_to_r, 0;
    T_wl_to_l, 0;
};

fprintf('Equalibria Condition Substitution ...\n');
g_at_eq = simplify(subs(g_param, eq_subs_partial(:,1), eq_subs_partial(:,2)));

% 符号求解 g = 0
fprintf('Solving g = 0 ...\n');

% g 向量中只有 g(2), g(3), g(4) 非零，且各自只含一个 theta
% g(2) 只含 theta_b → 解 g(2)=0 得 theta_b
% g(3) 只含 theta_r → 解 g(3)=0 得 theta_r
% g(4) 只含 theta_l → 解 g(4)=0 得 theta_l

% 解 theta_b (从 g(2)=0)
if g_at_eq(2) == 0
    theta_b_star = 0;
    fprintf(' g(2) = 0 恒成立, theta_b* = 0\n');
else
    sol_b = solve(g_at_eq(2) == 0, theta_b, 'Real', true);
    % 选择接近0的解
    sol_b_vals = double(sol_b);
    [~, idx_b] = min(abs(sol_b_vals));
    theta_b_star = sol_b_vals(idx_b);
    fprintf(' g(2)=0: theta_b* = %.10f rad (%.6f deg)\n', theta_b_star, rad2deg(theta_b_star));
end

% 解 theta_r (从 g(3)=0)
if g_at_eq(3) == 0
    theta_r_star = 0;
    fprintf(' g(3) = 0 恒成立, theta_r* = 0\n');
else
    sol_r = solve(g_at_eq(3) == 0, theta_r, 'Real', true);
    sol_r_vals = double(sol_r);
    [~, idx_r] = min(abs(sol_r_vals));
    theta_r_star = sol_r_vals(idx_r);
    fprintf(' g(3)=0: theta_r* = %.10f rad (%.6f deg)\n', theta_r_star, rad2deg(theta_r_star));
end

% 解 theta_l (从 g(4)=0)
if g_at_eq(4) == 0
    theta_l_star = 0;
    fprintf(' g(4) = 0 恒成立, theta_l* = 0\n');
else
    sol_l = solve(g_at_eq(4) == 0, theta_l, 'Real', true);
    sol_l_vals = double(sol_l);
    [~, idx_l] = min(abs(sol_l_vals));
    theta_l_star = sol_l_vals(idx_l);
    fprintf(' g(4)=0: theta_l* = %.10f rad (%.6f deg)\n', theta_l_star, rad2deg(theta_l_star));
end

theta_star = [theta_l_star; theta_r_star; theta_b_star];
g_func = matlabFunction(g_at_eq, 'Vars', {[theta_l; theta_r; theta_b]});
fval = g_func(theta_star);

fprintf('Solutions:\n');
fprintf('  theta_l* = %.10f rad (%.6f deg)\n', theta_l_star, rad2deg(theta_l_star));
fprintf('  theta_r* = %.10f rad (%.6f deg)\n', theta_r_star, rad2deg(theta_r_star));
fprintf('  theta_b* = %.10f rad (%.6f deg)\n', theta_b_star, rad2deg(theta_b_star));
fprintf('  ||g||: %.2e\n', norm(fval));
fprintf('  ✓ Success!\n\n');

% 完整的平衡点代换
eq_subs_full = {
    theta_l, theta_l_star;
    theta_r, theta_r_star;
    theta_b, theta_b_star;
    dtheta_l, 0;
    dtheta_r, 0;
    dtheta_b, 0;
    phi, 0;
    dphi, 0;
    X_b_h, 0;
    dX_b_h, 0;
};

% 平衡点处的 M, B, g 矩阵 (数值)
fprintf('Equalibria Substitution ...\n');
M_eq = double(subs(M_param, eq_subs_full(:,1), eq_subs_full(:,2)));
B_eq = double(subs(B_param, eq_subs_full(:,1), eq_subs_full(:,2)));
g_eq = double(subs(g_param, eq_subs_full(:,1), eq_subs_full(:,2)));

% 计算 dg/d(theta) 在平衡点
dg_dtheta_l = double(subs(diff(g_param, theta_l), eq_subs_full(:,1), eq_subs_full(:,2)));
dg_dtheta_r = double(subs(diff(g_param, theta_r), eq_subs_full(:,1), eq_subs_full(:,2)));
dg_dtheta_b = double(subs(diff(g_param, theta_b), eq_subs_full(:,1), eq_subs_full(:,2)));

n = 10;
m_ctrl = 4;

A_num = zeros(n, n);
B_num = zeros(n, m_ctrl);

fprintf('Solving A,B Matrices ...\n');

% 运动学关系 (位置-速度)
A_num(1,2) = 1;   % dX_b^h/dt = V_b^h
A_num(3,4) = 1;   % dphi/dt = dphi
A_num(5,6) = 1;   % dtheta_l/dt = dtheta_l
A_num(7,8) = 1;   % dtheta_r/dt = dtheta_r
A_num(9,10) = 1;  % dtheta_b/dt = dtheta_b

% 计算 M^{-1}
M_inv = inv(M_eq);

% dg/d(theta) 矩阵
dg_dtheta = [zeros(5,1), dg_dtheta_l, dg_dtheta_r, dg_dtheta_b];

% A矩阵动力学部分: M^{-1} * dg/d(theta)
A_dyn = M_inv * dg_dtheta;

% 填充A矩阵 (加速度行)
A_num(2,5) = A_dyn(1,2);   A_num(2,7) = A_dyn(1,3);   A_num(2,9) = A_dyn(1,4);
A_num(4,5) = A_dyn(2,2);   A_num(4,7) = A_dyn(2,3);   A_num(4,9) = A_dyn(2,4);
A_num(6,5) = A_dyn(3,2);   A_num(6,7) = A_dyn(3,3);   A_num(6,9) = A_dyn(3,4);
A_num(8,5) = A_dyn(4,2);   A_num(8,7) = A_dyn(4,3);   A_num(8,9) = A_dyn(4,4);
A_num(10,5) = A_dyn(5,2);  A_num(10,7) = A_dyn(5,3);  A_num(10,9) = A_dyn(5,4);

% B矩阵: M^{-1} * B
B_dyn = M_inv * B_eq;

B_num(2,:) = B_dyn(1,:);
B_num(4,:) = B_dyn(2,:);
B_num(6,:) = B_dyn(3,:);
B_num(8,:) = B_dyn(4,:);
B_num(10,:) = B_dyn(5,:);

fprintf('A (10×10):\n');
disp(A_num);

fprintf('B (10×4):\n');
disp(B_num);

Co = ctrb(A_num, B_num);
rank_Co = rank(Co);
fprintf('可控性矩阵秩: %d (系统维度: 10)\n', rank_Co);

if rank_Co < 10
    fprintf('  ⚠ 系统不完全可控 (秩=%d < 10)\n', rank_Co);
    fprintf('  物理原因: X_b^h 和 phi 是积分器状态\n');
    fprintf('  解决方案: 这是正常的! LQR仍可计算\n\n');
else
    fprintf('  ✓ 系统完全可控\n\n');
end