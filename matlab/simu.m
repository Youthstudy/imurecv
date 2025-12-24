% =========================================================================
% 混合阻抗/导纳控制系统仿真 - 复现Ott等人2015年论文
% =========================================================================
clear all; close all; clc;

%% 1. 系统参数设置
% 系统参数
m = 1.0;              % 实际质量 (kg)
m_hat = 0.8;          % 估计质量 (kg)
c_v = 1.0;            % 粘性摩擦系数 (Ns/m)
F_c = 3.0;            % 库伦摩擦力 (N)

% 期望阻抗参数
M_d = 1.0;            % 期望惯性 (kg)
K_d = 100;            % 期望刚度 (N/m)
D_d = 2*0.7*sqrt(K_d*M_d);  % 期望阻尼 (Ns/m)

% 位置控制器参数
k_p = 1e6;            % 位置增益 (N/m)
k_v = 2*0.7*sqrt(k_p*m);    % 速度增益 (Ns/m)

% 仿真参数
T = 0.001;            % 采样时间 (s)
T_d = 0.002;          % 时间延迟 (s)
t_sim = 1.0;          % 仿真时间 (s)
time = 0:T:t_sim;
N = length(time);

% 噪声参数
noise_std = 0.1;      % 力传感器噪声标准差

%% 2. 环境刚度设置
k_e_soft = 10;        % 软环境 (N/m)
k_e_medium = 300;     % 中等环境 (N/m)
k_e_stiff = 3200;     % 硬环境 (N/m)

%% 3. 混合控制参数
delta = 0.010;        % 切换周期 (s)
n_values = [0, 0.2, 0.4, 0.6, 0.8, 1.0];  % 占空比



%% 8. 仿真所有场景
fprintf('开始仿真...\n');

% 虚拟平衡位置
x0 = 1.0;

environments = struct('name', {'软环境', '中等环境', '硬环境'}, ...
                      'k_e', {k_e_soft, k_e_medium, k_e_stiff});

% 存储结果
results = struct();

for env_idx = 1:length(environments)
    env_name = environments(env_idx).name;
    k_e = environments(env_idx).k_e;
    
    fprintf('仿真 %s (k_e = %.1f N/m)...\n', env_name, k_e);
    
    % 计算参考轨迹
    x_ref = arrayfun(@(t) compute_reference(t, K_d, M_d, D_d, k_e, x0), time);
    
    % 阻抗控制
    [x_imp, ~, F_imp] = impedance_control(k_e, m, m_hat, K_d, D_d, M_d, ...
                                          c_v, F_c, T, time, x0, noise_std, T_d);
    
    % 导纳控制
    [x_adm, ~, F_adm] = admittance_control(k_e, m, m_hat, K_d, D_d, M_d, ...
                                           k_p, k_v, c_v, F_c, T, time, x0, ...
                                           noise_std, T_d);
    
    % 混合控制(不同占空比)
    x_hyb = cell(length(n_values), 1);
    F_hyb = cell(length(n_values), 1);
    for n_idx = 1:length(n_values)
        n = n_values(n_idx);
        [x_hyb{n_idx}, ~, F_hyb{n_idx}] = hybrid_control(k_e, m, m_hat, K_d, D_d, M_d, ...
                                                          k_p, k_v, c_v, F_c, T, time, x0, ...
                                                          noise_std, T_d, delta, n);
    end
    
    % 保存结果
    results(env_idx).name = env_name;
    results(env_idx).k_e = k_e;
    results(env_idx).x_ref = x_ref;
    results(env_idx).x_imp = x_imp;
    results(env_idx).x_adm = x_adm;
    results(env_idx).F_imp = F_imp;
    results(env_idx).F_adm = F_adm;
    results(env_idx).x_hyb = x_hyb;
    results(env_idx).F_hyb = F_hyb;
end

%% 9. 绘图
fprintf('绘制图表...\n');

% 图6: 理想轨迹
figure('Name', '图6: 理想轨迹', 'Position', [100 100 800 500]);
hold on; grid on;
for env_idx = 1:length(environments)
    k_e = results(env_idx).k_e;
    x_ref_normalized = results(env_idx).x_ref * (K_d + k_e) / K_d;
    plot(time, x_ref_normalized, 'LineWidth', 2, 'DisplayName', results(env_idx).name);
end
xlabel('时间 (s)', 'FontSize', 12);
ylabel('归一化位置', 'FontSize', 12);
title('理想轨迹 (不同环境刚度)', 'FontSize', 14);
legend('Location', 'best');
xlim([0 1]);

% 图7-9: 轨迹误差对比
for env_idx = 1:length(environments)
    fig_num = 6 + env_idx;
    figure('Name', sprintf('图%d: %s轨迹误差', fig_num, results(env_idx).name), ...
           'Position', [100+env_idx*50 100+env_idx*50 800 500]);
    hold on; grid on;
    
    x_ref = results(env_idx).x_ref;
    plot(time, results(env_idx).x_imp - x_ref, 'b-', 'LineWidth', 2, 'DisplayName', '阻抗控制');
    plot(time, results(env_idx).x_adm - x_ref, 'r-', 'LineWidth', 2, 'DisplayName', '导纳控制');
    
    xlabel('时间 (s)', 'FontSize', 12);
    ylabel('轨迹误差 (m)', 'FontSize', 12);
    title(sprintf('%s - 轨迹误差对比', results(env_idx).name), 'FontSize', 14);
    legend('Location', 'best');
    xlim([0 1]);
end

% 图15-17: 混合控制性能
for env_idx = 1:length(environments)
    fig_num = 14 + env_idx;
    figure('Name', sprintf('图%d: %s混合控制', fig_num, results(env_idx).name), ...
           'Position', [150+env_idx*50 150+env_idx*50 800 500]);
    hold on; grid on;
    
    x_ref = results(env_idx).x_ref;
    
    % 阻抗和导纳控制
    plot(time, results(env_idx).x_imp - x_ref, 'b--', 'LineWidth', 1.5, 'DisplayName', '阻抗控制');
    plot(time, results(env_idx).x_adm - x_ref, 'r--', 'LineWidth', 1.5, 'DisplayName', '导纳控制');
    
    % 混合控制(跳过n=0和n=1)
    colors = {'g', 'c', 'm', 'k'};
    for n_idx = 2:length(n_values)-1
        n = n_values(n_idx);
        color = colors{n_idx-1};
        plot(time, results(env_idx).x_hyb{n_idx} - x_ref, ...
             [color '-'], 'LineWidth', 2, 'DisplayName', sprintf('混合 n=%.1f', n));
    end
    
    xlabel('时间 (s)', 'FontSize', 12);
    ylabel('轨迹误差 (m)', 'FontSize', 12);
    title(sprintf('%s - 混合控制性能', results(env_idx).name), 'FontSize', 14);
    legend('Location', 'best');
    xlim([0 1]);
    grid on;
end

% 控制力输出对比图
for env_idx = 1:length(environments)
    figure('Name', sprintf('%s - 控制力输出', results(env_idx).name), ...
           'Position', [200+env_idx*50 200+env_idx*50 800 500]);
    hold on; grid on;
    
    plot(time, results(env_idx).F_imp, 'b-', 'LineWidth', 2, 'DisplayName', '阻抗控制');
    plot(time, results(env_idx).F_adm, 'r-', 'LineWidth', 2, 'DisplayName', '导纳控制');
    
    % 选择几个混合控制力输出
    plot(time, results(env_idx).F_hyb{3}, 'g-', 'LineWidth', 2, 'DisplayName', '混合 n=0.4');
    plot(time, results(env_idx).F_hyb{4}, 'm-', 'LineWidth', 2, 'DisplayName', '混合 n=0.6');
    
    xlabel('时间 (s)', 'FontSize', 12);
    ylabel('控制力 (N)', 'FontSize', 12);
    title(sprintf('%s - 控制力输出对比', results(env_idx).name), 'FontSize', 14);
    legend('Location', 'best');
    xlim([0 1]);
    grid on;
end

fprintf('仿真完成!\n');
fprintf('生成了所有图表,包括:\n');
fprintf('- 图6: 理想轨迹\n');
fprintf('- 图7-9: 不同环境下的轨迹误差对比\n');
fprintf('- 图15-17: 混合控制性能\n');
fprintf('- 控制力输出对比图\n');

%% 4. 参考轨迹生成函数
function x_ref = compute_reference(t, K_d, M_d, D_d, k_e, x0)
    % 计算理想响应
    zeta = D_d/(2*sqrt(M_d*(K_d+k_e)));
    omega_n = sqrt((K_d+k_e)/M_d);
    omega_d = omega_n*sqrt(1-zeta^2);
    
    x_ref = (K_d/(K_d+k_e))*x0*(1 - exp(-zeta*omega_n*t).*(cos(omega_d*t) + ...
            (zeta*omega_n/omega_d)*sin(omega_d*t)));
end

%% 5. 阻抗控制实现
function [x, x_dot, F_control] = impedance_control(k_e, m, m_hat, K_d, D_d, M_d, ...
                                          c_v, F_c, T, time, x0, noise_std, T_d)
    N = length(time);
    x = zeros(1, N);
    x_dot = zeros(1, N);
    x_ddot = zeros(1, N);
    F_control = zeros(1, N);
    F_ext = zeros(1, N);
    
    % 初始接触
    x(1) = 0;
    x_dot(1) = 0;
    
    for i = 2:N
        % 误差
        e = x(i-1) - x0;
        e_dot = x_dot(i-1);
        
        % 外力(环境力 + 摩擦力)
        F_ext(i-1) = -k_e * e;
        F_friction = -sign(x_dot(i-1))*(c_v*abs(x_dot(i-1)) + F_c);
        
        % 添加测量噪声和延迟
        delay_samples = round(T_d/T);
        idx = max(1, i-delay_samples);
        F_ext_measured = F_ext(idx) + noise_std*randn();
        
        % 阻抗控制律 (Eq. 6)
        F_control(i-1) = (m_hat/M_d - 1)*F_ext_measured - ...
                         (m_hat/M_d)*(D_d*e_dot + K_d*e);
        
        % 系统动力学
        F_total = F_control(i-1) + F_ext(i-1) + F_friction;
        x_ddot(i-1) = F_total/m;
        
        % 积分
        x_dot(i) = x_dot(i-1) + x_ddot(i-1)*T;
        x(i) = x(i-1) + x_dot(i-1)*T;
    end
    
    F_control(N) = F_control(N-1);
end

%% 6. 导纳控制实现
function [x, x_dot, F_control] = admittance_control(k_e, m, m_hat, K_d, D_d, M_d, ...
                                            k_p, k_v, c_v, F_c, T, time, x0, noise_std, T_d)
    N = length(time);
    x = zeros(1, N);
    x_dot = zeros(1, N);
    x_d = zeros(1, N);
    x_d_dot = zeros(1, N);
    F_control = zeros(1, N);
    F_ext = zeros(1, N);
    
    % 初始条件
    x(1) = 0;
    x_dot(1) = 0;
    x_d(1) = 0;
    x_d_dot(1) = 0;
    
    for i = 2:N
        % 误差
        e_d = x_d(i-1) - x0;
        e_d_dot = x_d_dot(i-1);
        
        % 外力
        e = x(i-1) - x0;
        F_ext(i-1) = -k_e * e;
        F_friction = -sign(x_dot(i-1))*(c_v*abs(x_dot(i-1)) + F_c);
        
        % 添加测量噪声和延迟
        delay_samples = round(T_d/T);
        idx = max(1, i-delay_samples);
        F_ext_measured = F_ext(idx) + noise_std*randn();
        
        % 导纳控制律 (Eq. 9)
        x_d_ddot = (F_ext_measured - D_d*e_d_dot - K_d*e_d)/M_d;
        
        % 位置控制器 (Eq. 7)
        F_control(i-1) = k_p*(x_d(i-1) - x(i-1)) - k_v*x_dot(i-1);
        
        % 系统动力学
        F_total = F_control(i-1) + F_ext(i-1) + F_friction;
        x_ddot = F_total/m;
        
        % 积分
        x_dot(i) = x_dot(i-1) + x_ddot*T;
        x(i) = x(i-1) + x_dot(i-1)*T;
        
        x_d_dot(i) = x_d_dot(i-1) + x_d_ddot*T;
        x_d(i) = x_d(i-1) + x_d_dot(i-1)*T;
    end
    
    F_control(N) = F_control(N-1);
end

%% 7. 混合控制实现
function [x, x_dot, F_control] = hybrid_control(k_e, m, m_hat, K_d, D_d, M_d, ...
                                        k_p, k_v, c_v, F_c, T, time, x0, ...
                                        noise_std, T_d, delta, n)
    N = length(time);
    x = zeros(1, N);
    x_dot = zeros(1, N);
    x_ddot = zeros(1, N);
    x_d = zeros(1, N);
    x_d_dot = zeros(1, N);
    F_control = zeros(1, N);
    F_ext = zeros(1, N);
    mode = zeros(1, N);  % 0=阻抗, 1=导纳
    
    % 初始条件
    x(1) = 0;
    x_dot(1) = 0;
    x_ddot(1) = 0;
    x_d(1) = 0;
    x_d_dot(1) = 0;
    
    k = 0;  % 周期计数器
    
    for i = 2:N
        t = time(i);
        
        % 确定当前模式
        t_in_period = mod(t, delta);
        if t_in_period <= (1-n)*delta
            current_mode = 0;  % 阻抗控制
        else
            current_mode = 1;  % 导纳控制
        end
        mode(i) = current_mode;
        
        % 检测模式切换
        if i > 1 && mode(i) ~= mode(i-1)
            if current_mode == 1  % 切换到导纳控制 - 使用完整状态映射
                % 状态映射 S_ai (Eq. 18-19)
                e = x(i-1) - x0;
                e_dot = x_dot(i-1);
                
                % 添加延迟和噪声的外力
                delay_samples = round(T_d/T);
                idx = max(1, i-1-delay_samples);
                F_ext_measured = F_ext(idx) + noise_std*randn();
                
                % 计算切换前的阻抗控制力
                F_imp = (m_hat/M_d - 1)*F_ext_measured - ...
                        (m_hat/M_d)*(D_d*e_dot + K_d*e);
                
                % 根据公式(18)初始化期望轨迹
                % e_d = e + (1/k_p)(F + k_v*ė)
                e_d_new = e + (F_imp + k_v*e_dot)/k_p;
                x_d(i-1) = e_d_new + x0;
                
                % ė_d = ė + (1/k_p)[Ḟ + (k_v/m)(F + F_ext)]
                F_dot = -(m_hat/M_d)*(D_d*x_ddot(i-1) + K_d*e_dot);
                e_d_dot_new = e_dot + (F_dot + (k_v/m)*(F_imp + F_ext(i-1)))/k_p;
                x_d_dot(i-1) = e_d_dot_new;
            end
        end
        
        % 外力
        e = x(i-1) - x0;
        F_ext(i-1) = -k_e * e;
        F_friction = -sign(x_dot(i-1))*(c_v*abs(x_dot(i-1)) + F_c);
        
        % 添加测量噪声和延迟
        delay_samples = round(T_d/T);
        idx = max(1, i-delay_samples);
        F_ext_measured = F_ext(idx) + noise_std*randn();
        
        % 控制律
        if current_mode == 0  % 阻抗控制
            e_dot = x_dot(i-1);
            F_control(i-1) = (m_hat/M_d - 1)*F_ext_measured - ...
                             (m_hat/M_d)*(D_d*e_dot + K_d*e);
        else  % 导纳控制
            e_d = x_d(i-1) - x0;
            e_d_dot = x_d_dot(i-1);
            
            % 导纳律
            x_d_ddot = (F_ext_measured - D_d*e_d_dot - K_d*e_d)/M_d;
            
            % 位置控制
            F_control(i-1) = k_p*(x_d(i-1) - x(i-1)) - k_v*x_dot(i-1);
            
            % 更新期望轨迹
            x_d_dot(i) = x_d_dot(i-1) + x_d_ddot*T;
            x_d(i) = x_d(i-1) + x_d_dot(i-1)*T;
        end
        
        % 系统动力学
        F_total = F_control(i-1) + F_ext(i-1) + F_friction;
        x_ddot(i-1) = F_total/m;
        
        % 积分
        x_dot(i) = x_dot(i-1) + x_ddot(i-1)*T;
        x(i) = x(i-1) + x_dot(i-1)*T;
    end
    
    F_control(N) = F_control(N-1);
    x_ddot(N) = x_ddot(N-1);
end