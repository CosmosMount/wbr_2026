%% VMC 垂直姿态分析 - 多推力工况并排对比
clear; clc;

% --- 1. 结构几何参数 ---
L1 = 0.22; 
L2 = 0.25; 
d_jac = 0.05; 
VMC_MotorDistance = 0.0; % 基座重合
VMC_HalfMotorDistance = VMC_MotorDistance / 2;

% --- 2. 采样范围设定 ---
n_points = 80;
phi1_vec = linspace(1.8, 2.6, n_points); 
phi4_vec = linspace(0.5, 1.3, n_points); 
[PHI1, PHI4] = meshgrid(phi1_vec, phi4_vec);

Fs_list = [250, 300, 450]; 
phi0_target = 90 * pi/180;
phi0_margin = 5 * pi/180; 

figure('Color', 'w', 'Name', '不同推力下 phi0 与扭矩关系对比', 'Position', [100, 100, 1200, 700]);

for col = 1:3
    Fs = Fs_list(col);
    
    % 数据收集
    PHI0_filtered = [];
    T1_filtered = [];
    T4_filtered = [];

    for i = 1:size(PHI1, 1)
        for j = 1:size(PHI1, 2)
            p1 = PHI1(i,j); p4 = PHI4(i,j);
            
            % VMC 角度解算
            xdb = VMC_MotorDistance + L1 * (cos(p4) - cos(p1));
            ydb = L1 * (sin(p4) - sin(p1));
            A0 = 2 * L2 * xdb; B0 = 2 * L2 * ydb; C0 = xdb^2 + ydb^2;
            delta = A0^2 + B0^2 - C0^2;
            if delta < 1e-4, continue; end

            phi2 = 2.0 * atan2((B0 + sqrt(delta)), (A0 + C0));
            cB = [L1 * cos(p1) - VMC_HalfMotorDistance; L1 * sin(p1)];
            cC = [cB(1) + L2 * cos(phi2); cB(2) + L2 * sin(phi2)];
            phi0 = atan2(cC(2), cC(1));

            % 垂直姿态过滤
            if abs(phi0 - phi0_target) > phi0_margin, continue; end

            % 雅可比力矩计算 (使用你提供的公式)
            cD = [L1 * cos(p4) + VMC_HalfMotorDistance; L1 * sin(p4)];
            phi3 = pi + atan2((cD(2) - cC(2)), (cD(1) - cC(1)));
            denom = sin(phi2 - phi3);
            if abs(denom) < 0.05, continue; end

            J11 = L1 * sin(p1 - phi2) * sin(phi3) / denom;
            J12 = (d_jac / L2) * sin(phi3 - p4) * sin(phi2) / denom;
            J21 = -L1 * sin(p1 - phi2) * cos(phi3) / denom;
            J22 = (d_jac / L2) * sin(phi3 - p4) * cos(phi2) / denom;
            
            % F_virtual 平行于 L2: Fx = Fs*cos(phi2), Fy = Fs*sin(phi2)
            Fx = -Fs * cos(phi2); Fy = Fs * sin(phi2);
            t1 = J11 * Fx + J21 * Fy;
            t4 = J12 * Fx + J22 * Fy;

            PHI0_filtered(end+1) = phi0 * 180/pi;
            T1_filtered(end+1) = t1;
            T4_filtered(end+1) = t4;
        end
    end

    % --- 绘制 T1 (第一行) ---
    subplot(2, 3, col);
    scatter(PHI0_filtered, T1_filtered, 8, 'b', 'filled');
    grid on; xlim([85 95]);
    title(['\tau_1 (Fs = ', num2str(Fs), ' N)']);
    if col == 1, ylabel('Torque \tau_1 (Nm)'); end

    % --- 绘制 T4 (第二行) ---
    subplot(2, 3, col + 3);
    scatter(PHI0_filtered, T4_filtered, 8, 'r', 'filled');
    grid on; xlim([85 95]);
    title(['\tau_4 (Fs = ', num2str(Fs), ' N)']);
    xlabel('\phi_0 (deg)');
    if col == 1, ylabel('Torque \tau_4 (Nm)'); end
end