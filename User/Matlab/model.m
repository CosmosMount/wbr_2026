clear;

% 物理常数
syms g real

% 机体常量 (去掉了公共的 Il，新增 Ill 和 Ilr)
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

%% ========================================
%  § 运动学约束
%  ========================================

a_l_h =  ll*cos(thetall)*ddthetall-ll*sin(thetall)*dthetall^2 ...
       + lr*cos(thetalr)*ddthetalr-lr*sin(thetalr)*dthetalr^2;

a_ll_v = -ll*sin(thetall)*ddthetall-thetall*cos(thetall)*dthetall^2;
a_lr_v = -lr*sin(thetalr)*ddthetalr-thetalr*cos(thetalr)*dthetalr^2;
    
ddthetawl = (ddx-Rb*ddphi+1/2*a_l_h)/Rw;
ddthetawr = (ddx+Rb*ddphi+1/2*a_l_h)/Rw;


%% ========================================
%  § 动力学方程 
%  ========================================

% 水平动量方程
eqn1 = (Twl-Iw*ddthetawl+Twr-Iw*ddthetawr) - ((1/2*mb+ml+mw)*Rw*(ddthetawl+ddthetawr)+(1/2*mb+ml)*a_l_h)*Rw;

% 偏航转动方程
eqn2 = (Iphi*ddphi) - ((Twr-Iw*ddthetawr)-(Twl-Iw*ddthetawl)*Rb/Rw);

% 左腿转动方程 (替换为 Ill)
eqn3 = Tll-Twl-ml*g*dll*sin(thetall+thetall0) ...
       -((Twl-Iw)/Rw-ml*Rw)*ddthetawl*ll*sin(thetall) ...
       +(1/2*mb+ml)*(g+(a_ll_v+a_lr_v)/2)*ll*cos(thetall) ...
       - Ill*ddthetall;

% 右腿转动方程 (替换为 Ilr)
eqn4 = Tlr-Twr-ml*g*dlr*sin(thetalr+thetalr0) ...
       -((Twr-Iw)/Rw-ml*Rw)*ddthetawr*lr*sin(thetalr) ...
       +(1/2*mb+ml)*(g+(a_ll_v+a_lr_v)/2)*lr*cos(thetalr) ...
       - Ilr*ddthetalr;

% 机体转动方程
eqn5 = (-Tll-Tlr+mb*g*db*cos(thetab+thetab0)) - (Ib*ddthetab);


%% ========================================
%  § 平衡点解算 
%  ========================================

thetall_eq = atan2((1/2*mb+ml)*ll-ml*dll*sin(thetall0), ml*dll*cos(thetall0));
thetalr_eq = atan2((1/2*mb+ml)*lr-ml*dlr*sin(thetalr0), ml*dlr*cos(thetalr0));

%% ========================================
%  § 物理参数
%  ========================================

% ==================== 物理常数 ====================
g_val = 9.78;              % 重力加速度 (m/s^2)

% ==================== 几何参数 ====================
Rw_val = 0.055;            % 轮子半径 (m)
Rb_val = (0.455+0.445)/4;  % 轮距/2 (m)

% ==================== 机体参数 ====================
mb_val = 7.846;            % 机体质量 (kg)
Ib_val = 0.150;            % 机体俯仰转动惯量 (kg·m²)
db_val = 0.055;            % 机体质心到俯仰轴距离 (m)
Iphi_val = 0.465;          % 整体yaw轴转动惯量 (kg·m²)
thetab0_val = 0;           % 质心偏移角度 (rad)

% ==================== 轮子参数 ====================
mw_val = 0.19;              % 轮质量 (kg)
Iw_val = 0.000207897;       % 轮转动惯量 (kg·m²)

% ==================== 腿部参数 ====================
ml_val = 1.62;              % 腿质量 (kg)

% (由于 Ill 和 Ilr 要作为传参，这里从数值替换表中将其移除，并补上缺失的 thetab0)
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
eqns = {eqn1; eqn2; eqn3; eqn4; eqn5};

M_sym = sym(zeros(5,5));
for i = 1:5
    for j = 1:5
        M_sym(i,j) = diff(eqns{i}, ddq(j));
    end
end

B_sym = sym(zeros(5,4));
for i = 1:5
    for j = 1:4
        B_sym(i,j) = diff(eqns{i}, u(j));
    end
end

g_sym = sym(zeros(5,1));
for i = 1:5
    g_sym(i) = subs(eqns{i}, [ddq; dq; u], zeros(14,1));
end

dg_dthetall_sym = diff(g_sym, thetall);
dg_dthetalr_sym = diff(g_sym, thetalr);
dg_dthetab_sym  = diff(g_sym, thetab);

% 代入物理参数
M_param = simplify(subs(M_sym, params_subs(:,1), params_subs(:,2)));
B_param = simplify(subs(B_sym, params_subs(:,1), params_subs(:,2)));
dg_dthetall_param = simplify(subs(dg_dthetall_sym, params_subs(:,1), params_subs(:,2)));
dg_dthetalr_param = simplify(subs(dg_dthetalr_sym, params_subs(:,1), params_subs(:,2)));
dg_dthetab_param  = simplify(subs(dg_dthetab_sym, params_subs(:,1), params_subs(:,2)));

% 平衡点代换
eq_subs = {
    thetall, thetall_eq;
    thetalr, thetalr_eq;
    thetab, -thetab0_val;
    dthetall, 0;
    dthetalr, 0;
    dthetab, 0;
    phi, 0;
    dphi, 0;
    x, 0;
    dx, 0;
    ddx, 0;
    ddphi, 0;
    ddthetall, 0;
    ddthetalr, 0;
    ddthetab, 0;
    Twl, 0;
    Twr, 0;
    Tll, 0;
    Tlr, 0;
};

M_eq = simplify(subs(M_param, eq_subs(:,1), eq_subs(:,2)));
B_eq = simplify(subs(B_param, eq_subs(:,1), eq_subs(:,2)));

dg_dtheta_l = simplify(subs(dg_dthetall_param, eq_subs(:,1), eq_subs(:,2)));
dg_dtheta_r = simplify(subs(dg_dthetalr_param, eq_subs(:,1), eq_subs(:,2)));
dg_dtheta_b = simplify(subs(dg_dthetab_param, eq_subs(:,1), eq_subs(:,2)));

n = 10;
m_ctrl = 4;

A_num = sym(zeros(n, n));
B_num = sym(zeros(n, m_ctrl));

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
A_dyn = -M_inv*dg_dtheta;

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

% (新增传入 Ill 和 Ilr 作为外部输入变量)
fprintf('正在生成 matrices.m...\n');
matlabFunction(A_num, B_num, thetall_eq, thetalr_eq, ...
    'File', 'matrices', ...
    'Vars', {ll, lr, thetall0, thetalr0, dll, dlr, Ill, Ilr}, ...
    'Outputs', {'A', 'B', 'thetall_eq', 'thetalr_eq'});
fprintf('✓ 生成完成！\n');