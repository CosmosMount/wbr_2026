clear; 

Leg_data_l =   [0.10, 0.00320, 0.09680, 0.110;
                0.11, 0.01102, 0.09898, 0.114;
                0.12, 0.01884, 0.10116, 0.118;
                0.13, 0.02666, 0.10334, 0.122;
                0.14, 0.03448, 0.10552, 0.126;
                0.15, 0.0423,  0.10770,	0.130;
                0.16, 0.05012, 0.10988, 0.134;
                0.17, 0.05794, 0.11206, 0.138;
                0.18, 0.06576, 0.11424, 0.142;
                0.19, 0.07358, 0.11642, 0.146;
                0.20, 0.08140, 0.11860,	0.150;
                0.21, 0.08922, 0.12078, 0.154;
                0.22, 0.09704, 0.12296, 0.158;
                0.23, 0.10486, 0.12514, 0.162;
                0.24, 0.11268, 0.12732, 0.166;
                0.25, 0.12050, 0.12950,	0.170;
                0.26, 0.12832, 0.13168, 0.174;
                0.27, 0.13614, 0.13386, 0.178;
                0.28, 0.14396, 0.13604, 0.182;
                0.29, 0.15178, 0.13822, 0.186;
                0.30, 0.15960, 0.14040,	0.190;
                0.31, 0.16742, 0.14258, 0.194;
                0.32, 0.17524, 0.14476, 0.198;
                0.33, 0.18306, 0.14694, 0.202;
                0.34, 0.19088, 0.14912, 0.206;
                0.35, 0.19870, 0.15130,	0.210;];

Leg_data_r =   Leg_data_l;



% fprintf('\n\t//-K in Arm Math Matrix Order: K00 K01 K02 K03 K04 K05 K10 K11 K12 K13 K14 K15\n')

leg = 0.10:0.01:0.35;

% Q矩阵
%         s  ds yaw dyaw alphal dalphal alphar dalphar theta dtheta
% 短腿长参数
Q = diag([60 40 40 20 1600 70 1600 70 10000 80]);
% 长腿长参数
% Q = diag([40 30 50 50 600 50 600 50 3000 30]);
% 其中：
% s        : 自然坐标系下机器人水平方向移动距离，单位：m，ds为其导数
% yaw      ：机器人水平方向移动时yaw偏航角度，dphi为其导数
% alphal   ：左腿摆杆与竖直方向（自然坐标系z轴）夹角，dtheta_ll为其导数
% alphar   ：右腿摆杆与竖直方向（自然坐标系z轴）夹角，dtheta_lr为其导数
% theta    ：机体与自然坐标系水平夹角，dtheta_b为其导数

% R矩阵
%    T_wl    T_wr     T_bl     T_br
R = diag([10 10 1 1]);
% 其中：
% T_wl: 左侧驱动轮输出力矩
% T_wr：右侧驱动轮输出力矩
% T_bl：左侧髋关节输出力矩
% T_br：右腿髋关节输出力矩
% 单位皆为Nm


%左右不同腿长的所有组合，计算出对应的K
L_vals = 0.10:0.01:0.35;
R_vals = 0.10:0.01:0.35;

num_L = length(L_vals);
num_R = length(R_vals);
num_samples = num_L * num_R;

K_matrices = zeros(4, 10, num_samples);

sample_idx = 1;
for i = 1:num_L
    for j = 1:num_R
        L_length = L_vals(i);
        R_length = R_vals(j);

        l_l = L_length;
        L_row_index = round((L_length-0.10)/0.01 + 1);
        l_l_d = Leg_data_l(L_row_index,2);      % 左腿质心到轮轴距离                                 （单位：m）
        I_ll = Leg_data_l(L_row_index,4);       % 左腿摆杆转动惯量                                   （单位：kg m^2）

        l_r = R_length;
        R_row_index = round((R_length-0.10)/0.01 + 1);
        l_r_d = Leg_data_r(R_row_index,2);      % 右腿质心到轮轴距离                                 （单位：m）
        I_lr = Leg_data_r(R_row_index,4);       % 右腿摆杆转动惯量                                   （单位：kg m^2）

        J_A = matrixA(L_length, R_length, l_l_d, l_r_d, I_ll, I_lr);
        J_B = matrixB(L_length, R_length, l_l_d, l_r_d, I_ll, I_lr);

        A = fillA(J_A);
        B = fillB(J_B);

        K_matrices(:, :, sample_idx) = lqr(A, B, Q, R);
        sample_idx = sample_idx +1;
    end
end

% 通过多项式拟合得出每个K矩阵的6个拟合系数
poly_coeffs_save = zeros(4, 10, 6);

for i = 1:4
    for j = 1:10
        y = squeeze(K_matrices(i, j, :));
        % 最小二乘法
        [L_grid, R_grid] = meshgrid(L_vals, R_vals);
        L_grid = L_grid(:);
        R_grid = R_grid(:);
        X = [L_grid.^2, L_grid.*R_grid, R_grid.^2, L_grid, R_grid, ones(num_samples, 1)]; %注意这里次序，后面输出拟合参数顺序有改变

        coeffs = X \ y;
        poly_coeffs_save(i, j, :) = coeffs;

    end
end

syms L_length R_length
K = sym(zeros(4, 10));
% 输出当前QR矩阵
fprintf('\t/*Q = [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f] R = [%.2f, %.2f, %.2f, %.2f]*/\n', ...
          Q(1,1), Q(2,2), Q(3,3), Q(4,4), Q(5,5), Q(6,6),Q(7,7),Q(8,8),Q(9,9),Q(10,10),R(1,1),R(2,2),R(3,3),R(4,4));
fprintf('\t/* a1 + a2*L_len + a3*R_len + a4*L_len^2 + a5*L_len*R_len + a6*R_len^2 */\n');
for i = 1:4
    for j = 1:10
        % 提取拟合系数
        p00 = poly_coeffs_save(i, j, 6);  % 常数项
        p10 = poly_coeffs_save(i, j, 4);  % L 的线性项
        p01 = poly_coeffs_save(i, j, 5);  % R 的线性项
        p20 = poly_coeffs_save(i, j, 1);  % L^2 的项
        p11 = poly_coeffs_save(i, j, 2);  % L * R 的项
        p02 = poly_coeffs_save(i, j, 3);  % R^2 的项
        
        K(i,j) = p00 + p10*L_length + p01*R_length + p20*L_length^2 + p11*L_length*R_length + p02*R_length^2;

        % 输出与左右腿长相关的数组
        % fprintf('\tK(%.f,%.f) = %8.6f + %8.6f*L_len + %8.6f*R_len + %8.6f*L_len^2 + %8.6f*L_len*R_len + %8.6f*R_len^2; \n', ...
        %           i, j, p00, p10, p01, p20, p11, p02);
        fprintf('\t{ %8.6f , %8.6f, %8.6f, %8.6f, %8.6f, %8.6f}, \n', ...
             p00, p10, p01, p20, p11, p02);

    end
end

function A = fillA(J_A)
    % 初始化A矩阵为零矩阵
    A = zeros(10, 10); 

    for p = 5:2:9
        A_index = (p - 3) / 2;
        A(2, p) = J_A(1, A_index);  % dX_b_h
        A(4, p) = J_A(2, A_index);  % dphi
        A(6, p) = J_A(3, A_index);  % dtheta_l
        A(8, p) = J_A(4, A_index);  % dtheta_r
        A(10, p) = J_A(5, A_index); % dtheta_b
    end

    for r = 1:10
        if rem(r, 2) == 0
            A(r, 1) = 0; A(r, 2) = 0; A(r, 3) = 0; A(r, 4) = 0; 
            A(r, 6) = 0; A(r, 8) = 0; A(r, 10) = 0;
        else
            A(r, :) = zeros(1, 10);
            A(r, r + 1) = 1;
        end
    end
    A = double(A);
end

function B = fillB(J_B)
    B = zeros(10, 4);

    for h = 1:4
        B(2, h) = J_B(1, h);  % dX_b_h
        B(4, h) = J_B(2, h);  % dphi
        B(6, h) = J_B(3, h);  % dtheta_l
        B(8, h) = J_B(4, h);  % dtheta_r
        B(10, h) = J_B(5, h); % dtheta_b
    end

    for e = 1:2:9
        B(e, :) = zeros(1, 4);  % 每个偶数行的元素都设置为零
    end
    B = double(B);
end