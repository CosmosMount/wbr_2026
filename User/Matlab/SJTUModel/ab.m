clear;

R_w = 0.095;                                % 驱动轮半径                  （单位：m）
R_l = 0.5/2;                                % 两个驱动轮之间距离/2         （单位：m）
l_c = 0.037;                                % 机体质心到腿部关节中心点距离  （单位：m）
m_w = 1.405; m_l = 0.7682; m_b = 22.3;      % 驱动轮质量 腿部质量 机体质量  （单位：kg）
I_w = m_w*R_w^2 ;                           % 驱动轮转动惯量               （单位：kg m^2）
I_b = m_b*(0.50^2+0.16^2)/12.0;             %m_b*(0.40^2+0.13^2)/12.0;     % 机体转动惯量(自然坐标系法向)  （单位：kg m^2）
I_z = m_b*(0.50^2+0.35^2)/12.0;             %m_b*(0.40^2+0.325^2)/12.0;    % 机器人z轴转动惯量

% 定义其他独立变量并补充其导数
syms theta_wl   theta_wr   % 左右驱动轮转角
syms dtheta_wl  dtheta_wr 
syms ddtheta_wl ddtheta_wr ddtheta_ll ddtheta_lr ddtheta_b

% 定义状态向量
syms s ds phi dphi theta_ll dtheta_ll theta_lr dtheta_lr theta_b dtheta_b

% 定义控制向量
syms T_wl T_wr T_bl T_br

% 输入物理参数：重力加速度
g = 9.78;
syms l_l l_r
syms L_row_index l_wl l_bl I_ll R_row_index l_wr l_br I_lr

eqn1 = (I_w*l_l/R_w+m_w*R_w*l_l+m_l*R_w*l_bl)*ddtheta_wl+(m_l*l_wl*l_bl-I_ll)*ddtheta_ll+(m_l*l_wl+m_b*l_l/2)*g*theta_ll+T_bl-T_wl*(1+l_l/R_w)==0;
eqn2 = (I_w*l_r/R_w+m_w*R_w*l_r+m_l*R_w*l_br)*ddtheta_wr+(m_l*l_wr*l_br-I_lr)*ddtheta_lr+(m_l*l_wr+m_b*l_r/2)*g*theta_lr+T_br-T_wr*(1+l_r/R_w)==0;
eqn3 = -(m_w*R_w*R_w+I_w+m_l*R_w*R_w+m_b*R_w*R_w/2)*ddtheta_wl-(m_w*R_w*R_w+I_w+m_l*R_w*R_w+m_b*R_w*R_w/2)*ddtheta_wr-(m_l*R_w*l_wl+m_b*R_w*l_l/2)*ddtheta_ll-(m_l*R_w*l_wr+m_b*R_w*l_r/2)*ddtheta_lr+T_wl+T_wr==0;
eqn4 = (m_w*R_w*l_c+I_w*l_c/R_w+m_l*R_w*l_c)*ddtheta_wl+(m_w*R_w*l_c+I_w*l_c/R_w+m_l*R_w*l_c)*ddtheta_wr+m_l*l_wl*l_c*ddtheta_ll+m_l*l_wr*l_c*ddtheta_lr-I_b*ddtheta_b+m_b*g*l_c*theta_b-(T_wl+T_wr)*l_c/R_w-(T_bl+T_br)==0;
eqn5 = ((I_z*R_w)/(2*R_l)+I_w*R_l/R_w)*ddtheta_wl-((I_z*R_w)/(2*R_l)+I_w*R_l/R_w)*ddtheta_wr+(I_z*l_l)/(2*R_l)*ddtheta_ll-(I_z*l_r)/(2*R_l)*ddtheta_lr-T_wl*R_l/R_w+T_wr*R_l/R_w==0;

[ddtheta_wl,ddtheta_wr,ddtheta_ll,ddtheta_lr,ddtheta_b] = solve(eqn1,eqn2,eqn3,eqn4,eqn5,ddtheta_wl,ddtheta_wr,ddtheta_ll,ddtheta_lr,ddtheta_b);


% 通过计算雅可比矩阵的方法得出控制矩阵A，B所需要的全部偏导数
J_A = jacobian([ddtheta_wl,ddtheta_wr,ddtheta_ll,ddtheta_lr,ddtheta_b],[theta_ll,theta_lr,theta_b]);
J_B = jacobian([ddtheta_wl,ddtheta_wr,ddtheta_ll,ddtheta_lr,ddtheta_b],[T_wl,T_wr,T_bl,T_br]);
% 将符号表达式转化为数值计算的函数
matlabFunction(J_A, 'File', 'Amatrix', 'Vars', {l_l, l_r, l_wl, l_bl, I_ll, l_wr, l_br, I_lr});
matlabFunction(J_B, 'File', 'Bmatrix', 'Vars', {l_l, l_r, l_wl, l_bl, I_ll, l_wr, l_br, I_lr});