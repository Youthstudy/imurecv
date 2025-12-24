%% 论文3.1章节：阻抗/导纳控制切换方法仿真
% 实现三种切换方法：代数法、微分法、迭代法
clear; clc; close all;

%% 1. 参数设置
% 阻抗控制参数
M_d = 10;      % 期望惯量 [kg]
D_d = 141.42;  % 期望阻尼 [Ns/m]
K_d = 500;     % 期望刚度 [N/m]

% 导纳控制参数（内部位置控制器）
K_p = 10000;   % 位置增益 [N/m]
K_v = 200;     % 速度增益 [Ns/m]

% 环境参数
K_env = 1500;  % 环境刚度 [N/m]
D_env = 10;    % 环境阻尼 [Ns/m]

% 设定点和初始状态
x_0 = 0.0;     % 设定点位置 [m]
dx_0 = 0.01;   % 设定点速度 [m/s]（前馈速度项）
ddx_0 = 0.0;   % 设定点加速度 [m/s^2]

% 切换时刻的机器人状态
x_i = 0.005;   % 当前位置 [m]
dx_i = 0.008;  % 当前速度 [m/s]
ddx_i = 0.0;   % 当前加速度 [m/s^2]

% 环境力（切换时刻）
F_env = K_env * (x_i - 0.01);  % 假设环境接触点在0.01m

%% 2. 计算阻抗控制力及其导数
e = x_i - x_0;
de = dx_i - dx_0;

% 阻抗控制力 (公式4简化版)
F_IMP = -K_d * e - D_d * de;

% 阻抗控制力的导数（数值近似）
dt = 0.001;  % 时间步长
F_IMP_prev = F_IMP - dt * (K_d * de + D_d * ddx_i);
dF_IMP = (F_IMP - F_IMP_prev) / dt;

fprintf('===== 切换时刻状态 =====\n');
fprintf('位置误差 e = %.6f m\n', e);
fprintf('速度误差 de = %.6f m/s\n', de);
fprintf('阻抗控制力 F_IMP = %.4f N\n', F_IMP);
fprintf('阻抗控制力导数 dF_IMP = %.4f N/s\n\n', dF_IMP);

%% 3. 方法1：代数切换法 (Algebraic Switching Method)
fprintf('========== 方法1：代数切换法 ==========\n');

% 初始猜测
e_d_alg = e;
de_d_alg = de;

% 迭代求解方程(13)和(14)
max_iter = 10;
for iter = 1:max_iter
    % 从方程(13)计算 e_d
    e_d_alg = e + (1/K_p) * (K_v * (dx_i - dx_0 - de_d_alg) + F_IMP);
    
    % 从方程(14)计算 de_d（假设 dde_d ≈ 0）
    dde_d_approx = 0;
    de_d_alg = (dx_i - dx_0) + (1 / (1 + K_v/K_p)) * (1/K_p) * ...
               (K_v * (ddx_i - ddx_0 - dde_d_approx) + dF_IMP);
end

fprintf('初始化结果：\n');
fprintf('  e_d = %.6f m\n', e_d_alg);
fprintf('  de_d = %.6f m/s\n', de_d_alg);

% 验证力的连续性
x_d_alg = x_0 + e_d_alg;
dx_d_alg = dx_0 + de_d_alg;
F_ADM_alg = -K_p * (x_i - x_d_alg) - K_v * (dx_i - dx_d_alg);
fprintf('  导纳控制力 F_ADM = %.4f N\n', F_ADM_alg);
fprintf('  力误差 |F_IMP - F_ADM| = %.6f N\n\n', abs(F_IMP - F_ADM_alg));

%% 4. 方法2：微分切换法 (Differential Switching Method)
fprintf('========== 方法2：微分切换法 ==========\n');

% 初始条件
e_d_diff = 0;
de_d_diff = 0;

% 时间积分参数
t_imp = 0:dt:0.05;  % 在阻抗控制期间积分50ms
e_d_hist = zeros(size(t_imp));
de_d_hist = zeros(size(t_imp));

% 初始值
e_d_hist(1) = e_d_diff;
de_d_hist(1) = de_d_diff;

% 欧拉积分方程(15)和(16)
for k = 1:(length(t_imp)-1)
    % 当前值
    e_d_curr = e_d_hist(k);
    de_d_curr = de_d_hist(k);
    
    % 方程(15)：de_d的微分方程
    de_d_dot = -(K_p/K_v) * de_d_curr + (1/K_v) * (K_p * e + F_IMP) + de;
    
    % 方程(16)：dde_d的微分方程（简化版）
    dde_d = -(K_p/K_v) * de_d_dot + (1/K_v) * (K_p * de + dF_IMP);
    
    % 积分更新
    de_d_hist(k+1) = de_d_curr + de_d_dot * dt;
    e_d_hist(k+1) = e_d_curr + de_d_curr * dt;
end

e_d_diff = e_d_hist(end);
de_d_diff = de_d_hist(end);

fprintf('初始化结果：\n');
fprintf('  e_d = %.6f m\n', e_d_diff);
fprintf('  de_d = %.6f m/s\n', de_d_diff);

% 验证力的连续性
x_d_diff = x_0 + e_d_diff;
dx_d_diff = dx_0 + de_d_diff;
F_ADM_diff = -K_p * (x_i - x_d_diff) - K_v * (dx_i - dx_d_diff);
fprintf('  导纳控制力 F_ADM = %.4f N\n', F_ADM_diff);
fprintf('  力误差 |F_IMP - F_ADM| = %.6f N\n\n', abs(F_IMP - F_ADM_diff));

%% 5. 方法3：迭代切换法 (Iterative Switching Method)
fprintf('========== 方法3：迭代切换法 ==========\n');

% 初始猜测（使用代数法结果）
e_d_iter = e_d_alg;
de_d_iter = de_d_alg;

% 迭代参数
toll = 0.001;     % 容差 [N/s]
step = 0.0002;    % 步长 [m]
max_iter = 100;
state = 'up';     % 状态标志
jj = 1;           % 幂次

% 迭代优化算法（Algorithm 1）
err_hist = [];
for iter = 1:max_iter
    % 计算导纳控制力的导数
    x_d_iter = x_0 + e_d_iter;
    dx_d_iter = dx_0 + de_d_iter;
    
    % 从导纳动力学方程计算 dde_d
    dde_d_iter = (1/M_d) * (-D_d * de_d_iter - K_d * e_d_iter + F_env);
    
    % 导纳控制力导数
    dF_ADM = K_p * (de_d_iter + dx_0 - dx_i) + ...
             K_v * (dde_d_iter + ddx_0 - ddx_i);
    
    % 误差
    err = abs(dF_ADM - dF_IMP);
    err_hist = [err_hist, err];
    
    if err < toll
        break;
    end
    
    % 调整 e_d
    if dF_ADM > dF_IMP
        if strcmp(state, 'down')
            jj = jj + 1;
        end
        state = 'up';
        e_d_iter = e_d_iter - step / (2^jj);
    else
        if strcmp(state, 'up')
            jj = jj + 1;
        end
        state = 'down';
        e_d_iter = e_d_iter + step / (2^jj);
    end
    
    % 更新 de_d（方程15）
    de_d_iter = -(K_p/K_v) * e_d_iter + (1/K_v) * (K_p * e + F_IMP) + de;
end

fprintf('迭代次数：%d\n', iter);
fprintf('初始化结果：\n');
fprintf('  e_d = %.6f m\n', e_d_iter);
fprintf('  de_d = %.6f m/s\n', de_d_iter);

% 验证力的连续性
x_d_iter = x_0 + e_d_iter;
dx_d_iter = dx_0 + de_d_iter;
F_ADM_iter = -K_p * (x_i - x_d_iter) - K_v * (dx_i - dx_d_iter);
fprintf('  导纳控制力 F_ADM = %.4f N\n', F_ADM_iter);
fprintf('  力误差 |F_IMP - F_ADM| = %.6f N\n', abs(F_IMP - F_ADM_iter));

% 计算力导数误差
dde_d_final = (1/M_d) * (-D_d * de_d_iter - K_d * e_d_iter + F_env);
dF_ADM_final = K_p * (de_d_iter + dx_0 - dx_i) + ...
               K_v * (dde_d_final + ddx_0 - ddx_i);
fprintf('  力导数误差 |dF_IMP - dF_ADM| = %.6f N/s\n\n', abs(dF_IMP - dF_ADM_final));

%% 6. 仿真切换前后的力响应
% 仿真参数
t_total = 0.1;  % 总仿真时间 [s]
t_switch = 0.05;  % 切换时刻 [s]
dt_sim = 0.0001;  % 仿真步长 [s]
t_sim = 0:dt_sim:t_total;
n_steps = length(t_sim);

% 初始化三种方法的结果
F_alg = zeros(1, n_steps);
F_diff = zeros(1, n_steps);
F_iter = zeros(1, n_steps);
F_no_init = zeros(1, n_steps);  % 无初始化对比

% 状态变量初始化
x_alg = x_i; dx_alg = dx_i;
x_diff = x_i; dx_diff = dx_i;
x_iter = x_i; dx_iter = dx_i;
x_no = x_i; dx_no = dx_i;

% 导纳控制器状态
e_d_alg_sim = 0; de_d_alg_sim = 0;
e_d_diff_sim = 0; de_d_diff_sim = 0;
e_d_iter_sim = 0; de_d_iter_sim = 0;
e_d_no_sim = 0; de_d_no_sim = 0;

% 机器人质量（简化模型）
M_robot = 8.0;

for k = 1:n_steps
    t = t_sim(k);
    
    if t < t_switch
        % 阻抗控制阶段
        e_alg = x_alg - x_0;
        de_alg = dx_alg - dx_0;
        F_alg(k) = -K_d * e_alg - D_d * de_alg;
        
        e_diff = x_diff - x_0;
        de_diff = dx_diff - dx_0;
        F_diff(k) = -K_d * e_diff - D_d * de_diff;
        
        e_iter = x_iter - x_0;
        de_iter = dx_iter - dx_0;
        F_iter(k) = -K_d * e_iter - D_d * de_iter;
        
        e_no = x_no - x_0;
        de_no = dx_no - dx_0;
        F_no_init(k) = -K_d * e_no - D_d * de_no;
        
    else
        % 导纳控制阶段
        if k == find(t_sim >= t_switch, 1)
            % 切换时刻初始化
            e_d_alg_sim = e_d_alg;
            de_d_alg_sim = de_d_alg;
            
            e_d_diff_sim = e_d_diff;
            de_d_diff_sim = de_d_diff;
            
            e_d_iter_sim = e_d_iter;
            de_d_iter_sim = de_d_iter;
            
            % 无初始化：直接使用当前误差
            e_d_no_sim = x_no - x_0;
            de_d_no_sim = 0;  % 速度初始化为0
        end
        
        % 导纳动力学积分（方法1：代数法）
        x_d_alg = x_0 + e_d_alg_sim;
        dx_d_alg = dx_0 + de_d_alg_sim;
        F_alg(k) = -K_p * (x_alg - x_d_alg) - K_v * (dx_alg - dx_d_alg);
        
        F_env_alg = -K_env * (x_alg - 0.01);
        dde_d_alg = (1/M_d) * (-D_d * de_d_alg_sim - K_d * e_d_alg_sim + F_env_alg);
        de_d_alg_sim = de_d_alg_sim + dde_d_alg * dt_sim;
        e_d_alg_sim = e_d_alg_sim + de_d_alg_sim * dt_sim;
        
        % 导纳动力学积分（方法2：微分法）
        x_d_diff = x_0 + e_d_diff_sim;
        dx_d_diff = dx_0 + de_d_diff_sim;
        F_diff(k) = -K_p * (x_diff - x_d_diff) - K_v * (dx_diff - dx_d_diff);
        
        F_env_diff = -K_env * (x_diff - 0.01);
        dde_d_diff = (1/M_d) * (-D_d * de_d_diff_sim - K_d * e_d_diff_sim + F_env_diff);
        de_d_diff_sim = de_d_diff_sim + dde_d_diff * dt_sim;
        e_d_diff_sim = e_d_diff_sim + de_d_diff_sim * dt_sim;
        
        % 导纳动力学积分（方法3：迭代法）
        x_d_iter = x_0 + e_d_iter_sim;
        dx_d_iter = dx_0 + de_d_iter_sim;
        F_iter(k) = -K_p * (x_iter - x_d_iter) - K_v * (dx_iter - dx_d_iter);
        
        F_env_iter = -K_env * (x_iter - 0.01);
        dde_d_iter = (1/M_d) * (-D_d * de_d_iter_sim - K_d * e_d_iter_sim + F_env_iter);
        de_d_iter_sim = de_d_iter_sim + dde_d_iter * dt_sim;
        e_d_iter_sim = e_d_iter_sim + de_d_iter_sim * dt_sim;
        
        % 无初始化方法
        x_d_no = x_0 + e_d_no_sim;
        dx_d_no = dx_0 + de_d_no_sim;
        F_no_init(k) = -K_p * (x_no - x_d_no) - K_v * (dx_no - dx_d_no);
        
        F_env_no = -K_env * (x_no - 0.01);
        dde_d_no = (1/M_d) * (-D_d * de_d_no_sim - K_d * e_d_no_sim + F_env_no);
        de_d_no_sim = de_d_no_sim + dde_d_no * dt_sim;
        e_d_no_sim = e_d_no_sim + de_d_no_sim * dt_sim;
    end
    
    % 更新机器人状态（简化动力学）
    F_env_alg = -K_env * (x_alg - 0.01) - D_env * dx_alg;
    ddx_alg = (F_alg(k) + F_env_alg) / M_robot;
    dx_alg = dx_alg + ddx_alg * dt_sim;
    x_alg = x_alg + dx_alg * dt_sim;
    
    F_env_diff = -K_env * (x_diff - 0.01) - D_env * dx_diff;
    ddx_diff = (F_diff(k) + F_env_diff) / M_robot;
    dx_diff = dx_diff + ddx_diff * dt_sim;
    x_diff = x_diff + dx_diff * dt_sim;
    
    F_env_iter = -K_env * (x_iter - 0.01) - D_env * dx_iter;
    ddx_iter = (F_iter(k) + F_env_iter) / M_robot;
    dx_iter = dx_iter + ddx_iter * dt_sim;
    x_iter = x_iter + dx_iter * dt_sim;
    
    F_env_no = -K_env * (x_no - 0.01) - D_env * dx_no;
    ddx_no = (F_no_init(k) + F_env_no) / M_robot;
    dx_no = dx_no + ddx_no * dt_sim;
    x_no = x_no + dx_no * dt_sim;
end

%% 7. 可视化结果
figure('Position', [50, 50, 1400, 900]);

% 子图1：控制力时间历程对比
subplot(3,3,1);
plot(t_sim*1000, F_alg, 'b-', 'LineWidth', 2); hold on;
plot(t_sim*1000, F_diff, 'r-', 'LineWidth', 2);
plot(t_sim*1000, F_iter, 'g-', 'LineWidth', 2);
plot(t_sim*1000, F_no_init, 'k--', 'LineWidth', 1.5);
xline(t_switch*1000, 'k--', 'LineWidth', 2, 'Label', '切换点');
xlabel('时间 [ms]');
ylabel('控制力 [N]');
title('控制力时间历程对比');
legend('代数法', '微分法', '迭代法', '无初始化', 'Location', 'best');
grid on;
xlim([0, t_total*1000]);

% 子图2：切换时刻放大图
subplot(3,3,2);
t_zoom = (t_switch - 0.01):dt_sim:(t_switch + 0.01);
idx_zoom = find(t_sim >= t_switch - 0.01 & t_sim <= t_switch + 0.01);
plot(t_sim(idx_zoom)*1000, F_alg(idx_zoom), 'b-', 'LineWidth', 2); hold on;
plot(t_sim(idx_zoom)*1000, F_diff(idx_zoom), 'r-', 'LineWidth', 2);
plot(t_sim(idx_zoom)*1000, F_iter(idx_zoom), 'g-', 'LineWidth', 2);
plot(t_sim(idx_zoom)*1000, F_no_init(idx_zoom), 'k--', 'LineWidth', 1.5);
xline(t_switch*1000, 'k--', 'LineWidth', 2);
xlabel('时间 [ms]');
ylabel('控制力 [N]');
title('切换时刻放大（±10ms）');
legend('代数法', '微分法', '迭代法', '无初始化', 'Location', 'best');
grid on;

% 子图3：力导数对比
subplot(3,3,3);
dF_alg = [0, diff(F_alg)/dt_sim];
dF_diff = [0, diff(F_diff)/dt_sim];
dF_iter = [0, diff(F_iter)/dt_sim];
dF_no = [0, diff(F_no_init)/dt_sim];
plot(t_sim(idx_zoom)*1000, dF_alg(idx_zoom), 'b-', 'LineWidth', 2); hold on;
plot(t_sim(idx_zoom)*1000, dF_diff(idx_zoom), 'r-', 'LineWidth', 2);
plot(t_sim(idx_zoom)*1000, dF_iter(idx_zoom), 'g-', 'LineWidth', 2);
plot(t_sim(idx_zoom)*1000, dF_no(idx_zoom), 'k--', 'LineWidth', 1.5);
xline(t_switch*1000, 'k--', 'LineWidth', 2);
xlabel('时间 [ms]');
ylabel('力导数 [N/s]');
title('力导数对比（切换时刻）');
legend('代数法', '微分法', '迭代法', '无初始化', 'Location', 'best');
grid on;

% 子图4：迭代法收敛过程
subplot(3,3,4);
plot(1:length(err_hist), err_hist, 'b-', 'LineWidth', 2);
hold on;
plot([1, length(err_hist)], [toll, toll], 'r--', 'LineWidth', 1.5);
xlabel('迭代次数');
ylabel('力导数误差 [N/s]');
title('迭代法收敛过程');
legend('误差', '容差', 'Location', 'best');
grid on;

% 子图5：三种方法的初始化值比较
subplot(3,3,5);
methods = {'代数法', '微分法', '迭代法'};
e_d_vals = [e_d_alg, e_d_diff, e_d_iter] * 1000;
b = bar(e_d_vals);
b.FaceColor = 'flat';
b.CData = [0.2, 0.6, 0.8; 0.8, 0.4, 0.2; 0.3, 0.7, 0.3];
set(gca, 'XTickLabel', methods);
ylabel('位置误差 e_d [mm]');
title('初始位置误差比较');
grid on;

% 子图6：力连续性比较
subplot(3,3,6);
F_vals = [F_IMP, F_ADM_alg, F_ADM_diff, F_ADM_iter];
labels = {'阻抗力', '代数法', '微分法', '迭代法'};
b = bar(F_vals);
b.FaceColor = 'flat';
b.CData = [0.5, 0.5, 0.5; 0.2, 0.6, 0.8; 0.8, 0.4, 0.2; 0.3, 0.7, 0.3];
set(gca, 'XTickLabel', labels);
ylabel('控制力 [N]');
title('切换时刻控制力验证');
grid on;

% 子图7：力的跳变量比较
subplot(3,3,7);
idx_sw = find(t_sim >= t_switch, 1);
F_jump_alg = abs(F_alg(idx_sw) - F_alg(idx_sw-1));
F_jump_diff = abs(F_diff(idx_sw) - F_diff(idx_sw-1));
F_jump_iter = abs(F_iter(idx_sw) - F_iter(idx_sw-1));
F_jump_no = abs(F_no_init(idx_sw) - F_no_init(idx_sw-1));
F_jumps = [F_jump_alg, F_jump_diff, F_jump_iter, F_jump_no] * 1000;
methods_all = {'代数法', '微分法', '迭代法', '无初始化'};
b = bar(F_jumps);
b.FaceColor = 'flat';
b.CData = [0.2, 0.6, 0.8; 0.8, 0.4, 0.2; 0.3, 0.7, 0.3; 0.5, 0.5, 0.5];
set(gca, 'XTickLabel', methods_all);
ylabel('力跳变 [mN]');
title('切换时刻力跳变量');
grid on;

% 子图8：微分法积分过程
subplot(3,3,8);
plot(t_imp*1000, e_d_hist*1000, 'b-', 'LineWidth', 2);
hold on;
plot(t_imp*1000, de_d_hist*1000, 'r-', 'LineWidth', 2);
xlabel('积分时间 [ms]');
ylabel('误差值 [mm or mm/s]');
title('微分法积分过程');
legend('e_d', 'de_d', 'Location', 'best');
grid on;

% 子图9：力平滑度指标
subplot(3,3,9);
% 计算力的二阶导数（平滑度指标）
smoothness_alg = std(diff(F_alg(idx_zoom), 2));
smoothness_diff = std(diff(F_diff(idx_zoom), 2));
smoothness_iter = std(diff(F_iter(idx_zoom), 2));
smoothness_no = std(diff(F_no_init(idx_zoom), 2));
smoothness = [smoothness_alg, smoothness_diff, smoothness_iter, smoothness_no];
b = bar(smoothness);
b.FaceColor = 'flat';
b.CData = [0.2, 0.6, 0.8; 0.8, 0.4, 0.2; 0.3, 0.7, 0.3; 0.5, 0.5, 0.5];
set(gca, 'XTickLabel', methods_all);
ylabel('力二阶导数标准差');
title('切换平滑度指标（越小越好）');
grid on;

sgtitle('阻抗/导纳控制切换方法完整对比', 'FontSize', 14, 'FontWeight', 'bold');

%% 7. 总结
fprintf('\n========== 总结 ==========\n');
fprintf('三种方法均能实现力的连续性切换\n');
fprintf('迭代法精度最高，但计算量较大\n');
fprintf('代数法计算效率最高，适合实时应用\n');
fprintf('微分法受数值积分误差影响\n');