%% ========================================================================
%% 状态映射详细解释与可视化
%% ========================================================================
clear all; close all; clc;

%% 1. 理论背景
fprintf('=== 状态映射理论 ===\n\n');

fprintf('阻抗控制状态空间: X_i = [e; ė]     (2维)\n');
fprintf('导纳控制状态空间: X_a = [e; ė; e_d; ė_d]  (4维)\n\n');

%% 2. 参数设置
m = 1.0;
M_d = 1.0;
K_d = 100;
D_d = 2*0.7*sqrt(K_d*M_d);
k_p = 1e3;
k_v = 2*0.7*sqrt(k_p*m);
k_e = 300;  % 环境刚度

fprintf('系统参数:\n');
fprintf('  质量 m = %.2f kg\n', m);
fprintf('  期望刚度 K_d = %.1f N/m\n', K_d);
fprintf('  位置增益 k_p = %.0e N/m\n', k_p);
fprintf('  环境刚度 k_e = %.1f N/m\n\n', k_e);

%% 3. 状态映射矩阵 S_ai (从阻抗到导纳)
fprintf('=== 映射矩阵 S_ai (阻抗→导纳) ===\n\n');

% 计算 S 矩阵元素 (公式 20)
s11 = 1 - (k_e/k_p)*(m/M_d - 1) - (K_d/k_p)*(m/M_d);
s12 = k_v/k_p - (D_d/k_p)*(m/M_d);
s21 = -(m/M_d)*(K_d + k_e)*(k_v/m - D_d/M_d)/k_p;
s22 = 1 - (k_e/k_p)*(m/M_d - 1) - (D_d/M_d)*(k_v/k_p - (D_d/k_p)*(m/M_d)) - (K_d/k_p)*(m/M_d);

S = [s11, s12; s21, s22];

% 完整的 S_ai 矩阵 (公式 19)
S_ai = [eye(2); S];

fprintf('S_ai = \n');
fprintf('  [1    0   ]  ← e_new = e_old\n');
fprintf('  [0    1   ]  ← ė_new = ė_old\n');
fprintf('  [%.4f  %.4f]  ← e_d 通过映射计算\n', s11, s12);
fprintf('  [%.4f  %.4f]  ← ė_d 通过映射计算\n\n', s21, s22);

fprintf('物理意义:\n');
fprintf('  前两行: 实际位置/速度直接继承\n');
fprintf('  后两行: 期望轨迹根据力的连续性条件计算\n\n');

%% 4. 状态映射矩阵 S_ia (从导纳到阻抗)
fprintf('=== 映射矩阵 S_ia (导纳→阻抗) ===\n\n');

S_ia = [eye(2), zeros(2)];

fprintf('S_ia = [1 0 0 0]  ← e_new = e_old\n');
fprintf('       [0 1 0 0]  ← ė_new = ė_old\n\n');
fprintf('物理意义: 只保留实际状态，丢弃期望轨迹\n\n');

%% 5. 为什么需要状态映射？可视化演示
fprintf('=== 状态映射的必要性演示 ===\n\n');

% 仿真参数
T = 0.001;
t_switch = 0.1;  % 切换时刻
time = 0:T:0.3;
N = length(time);

% 初始化
x = zeros(1, N);
x_dot = zeros(1, N);
F_control_with_map = zeros(1, N);
F_control_without_map = zeros(1, N);
x0 = 1.0;

% 场景1: 有状态映射
x_d_with = zeros(1, N);
x_d_dot_with = zeros(1, N);

% 场景2: 无状态映射(粗暴切换)
x_d_without = zeros(1, N);
x_d_dot_without = zeros(1, N);

for i = 2:N
    t = time(i);
    e = x(i-1) - x0;
    e_dot = x_dot(i-1);
    F_ext = -k_e * e;
    
    if t < t_switch  % 阻抗控制阶段
        % 阻抗控制律
        F = (m/M_d - 1)*F_ext - (m/M_d)*(D_d*e_dot + K_d*e);
        F_control_with_map(i-1) = F;
        F_control_without_map(i-1) = F;
        
    elseif i == find(time >= t_switch, 1)  % 切换时刻
        fprintf('t = %.3f s: 控制器切换!\n', t);
        
        % 场景1: 使用状态映射
        X_i = [e; e_dot];
        X_a = S_ai * X_i;
        x_d_with(i-1) = X_a(3) + x0;
        x_d_dot_with(i-1) = X_a(4);
        
        e_d = x_d_with(i-1) - x0;
        e_d_dot = x_d_dot_with(i-1);
        x_d_ddot = (F_ext - D_d*e_d_dot - K_d*e_d)/M_d;
        x_d_dot_with(i) = x_d_dot_with(i-1) + x_d_ddot*T;
        x_d_with(i) = x_d_with(i-1) + x_d_dot_with(i-1)*T;
        
        F_with = k_p*(x_d_with(i-1) - x(i-1)) - k_v*x_dot(i-1);
        F_control_with_map(i-1) = F_with;
        
        % 场景2: 不使用映射，直接用当前位置
        x_d_without(i-1) = x(i-1);
        x_d_dot_without(i-1) = x_dot(i-1);
        
        e_d = x_d_without(i-1) - x0;
        e_d_dot = x_d_dot_without(i-1);
        x_d_ddot = (F_ext - D_d*e_d_dot - K_d*e_d)/M_d;
        x_d_dot_without(i) = x_d_dot_without(i-1) + x_d_ddot*T;
        x_d_without(i) = x_d_without(i-1) + x_d_dot_without(i-1)*T;
        
        F_without = k_p*(x_d_without(i-1) - x(i-1)) - k_v*x_dot(i-1);
        F_control_without_map(i-1) = F_without;
        
        fprintf('  阻抗控制力: %.2f N\n', F_control_with_map(i-2));
        fprintf('  有映射导纳力: %.2f N (误差: %.2f%%)\n', F_with, ...
                abs(F_with-F_control_with_map(i-2))/abs(F_control_with_map(i-2))*100);
        fprintf('  无映射导纳力: %.2f N (误差: %.2f%%)\n\n', F_without, ...
                abs(F_without-F_control_with_map(i-2))/abs(F_control_with_map(i-2))*100);
        
    else  % 导纳控制阶段
        % 场景1: 有映射
        e_d = x_d_with(i-1) - x0;
        e_d_dot = x_d_dot_with(i-1);
        x_d_ddot = (F_ext - D_d*e_d_dot - K_d*e_d)/M_d;
        x_d_dot_with(i) = x_d_dot_with(i-1) + x_d_ddot*T;
        x_d_with(i) = x_d_with(i-1) + x_d_dot_with(i-1)*T;
        F_control_with_map(i-1) = k_p*(x_d_with(i-1) - x(i-1)) - k_v*x_dot(i-1);
        
        % 场景2: 无映射
        e_d = x_d_without(i-1) - x0;
        e_d_dot = x_d_dot_without(i-1);
        x_d_ddot = (F_ext - D_d*e_d_dot - K_d*e_d)/M_d;
        x_d_dot_without(i) = x_d_dot_without(i-1) + x_d_ddot*T;
        x_d_without(i) = x_d_without(i-1) + x_d_dot_without(i-1)*T;
        F_control_without_map(i-1) = k_p*(x_d_without(i-1) - x(i-1)) - k_v*x_dot(i-1);
    end
    
    % 系统动力学(使用有映射的控制力)
    F_total = F_control_with_map(i-1) + F_ext;
    x_ddot = F_total/m;
    x_dot(i) = x_dot(i-1) + x_ddot*T;
    x(i) = x(i-1) + x_dot(i-1)*T;
end

F_control_with_map(N) = F_control_with_map(N-1);
F_control_without_map(N) = F_control_without_map(N-1);

%% 6. 绘图对比
figure('Position', [100 100 1200 800]);

% 子图1: 控制力对比
subplot(3,1,1);
hold on; grid on;
plot(time, F_control_with_map, 'b-', 'LineWidth', 2, 'DisplayName', '有状态映射');
plot(time, F_control_without_map, 'r--', 'LineWidth', 2, 'DisplayName', '无状态映射');
plot([t_switch t_switch], ylim, 'k--', 'LineWidth', 1.5, 'DisplayName', '切换时刻');
xlabel('时间 (s)');
ylabel('控制力 (N)');
title('控制力对比 - 状态映射的作用');
legend('Location', 'best');
xlim([0 0.3]);

% 子图2: 控制力差异(放大)
subplot(3,1,2);
hold on; grid on;
force_diff = abs(F_control_with_map - F_control_without_map);
plot(time, force_diff, 'm-', 'LineWidth', 2);
plot([t_switch t_switch], ylim, 'k--', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('控制力差异 (N)');
title('|有映射 - 无映射| 的控制力差异');
xlim([0 0.3]);

% 子图3: 放大切换时刻
subplot(3,1,3);
hold on; grid on;
idx_zoom = find(time >= t_switch-0.01 & time <= t_switch+0.01);
plot(time(idx_zoom), F_control_with_map(idx_zoom), 'b-', 'LineWidth', 2, 'DisplayName', '有状态映射');
plot(time(idx_zoom), F_control_without_map(idx_zoom), 'r--', 'LineWidth', 2, 'DisplayName', '无状态映射');
plot([t_switch t_switch], ylim, 'k--', 'LineWidth', 1.5, 'DisplayName', '切换时刻');
xlabel('时间 (s)');
ylabel('控制力 (N)');
title('切换时刻放大图 - 看清力的跳变');
legend('Location', 'best');
xlim([t_switch-0.01 t_switch+0.01]);

%% 7. 状态映射的作用总结
figure('Position', [150 150 1000 600]);

% 示意图
annotation('textbox', [0.1 0.7 0.8 0.2], 'String', {...
    '状态映射的三大作用:', ...
    '', ...
    '1. 保证控制力连续性 → 避免冲击和振动', ...
    '   • 有映射: 切换前后控制力平滑过渡', ...
    '   • 无映射: 切换时控制力跳变，可能引起系统震荡', ...
    '', ...
    '2. 维持系统稳定性 → 满足李雅普诺夫条件', ...
    '   • 通过确保能量函数单调递减', ...
    '   • 防止切换引入的能量突增', ...
    '', ...
    '3. 提高控制精度 → 优化瞬态响应', ...
    '   • 期望轨迹 x_d 在切换时得到合理初始化', ...
    '   • 而不是粗暴地设为当前位置'}, ...
    'FontSize', 12, 'FontWeight', 'bold', 'EdgeColor', 'k', 'LineWidth', 2);

% 公式展示
annotation('textbox', [0.1 0.35 0.8 0.3], 'String', {...
    '状态映射公式:', ...
    '', ...
    '从阻抗到导纳 (S_ai):', ...
    '  [e_new]       [1   0  ] [e_old  ]', ...
    '  [ė_new]   =   [0   1  ] [ė_old  ]', ...
    '  [e_d  ]       [s11 s12] ', ...
    '  [ė_d  ]       [s21 s22] ', ...
    '', ...
    '其中 S 矩阵由力连续性条件推导:', ...
    '  F_impedance(切换前) = F_admittance(切换后)', ...
    '', ...
    '从导纳到阻抗 (S_ia):', ...
    '  [e_new]   =  [1 0 0 0] [e_old  ]   → 只保留实际状态', ...
    '  [ė_new]      [0 1 0 0] [ė_old   ]', ...
    '                          [e_d    ]', ...
    '                          [ė_d    ]'}, ...
    'FontSize', 10, 'FontName', 'Courier', 'EdgeColor', 'b', 'LineWidth', 1.5);

%% 8. 数值验证
fprintf('=== 数值验证 ===\n\n');
fprintf('切换时刻的力连续性检验:\n');
idx_switch = find(time >= t_switch, 1);
fprintf('  切换前(阻抗): F = %.4f N\n', F_control_with_map(idx_switch-1));
fprintf('  切换后(导纳-有映射): F = %.4f N\n', F_control_with_map(idx_switch));
fprintf('  切换后(导纳-无映射): F = %.4f N\n', F_control_without_map(idx_switch));
fprintf('  有映射误差: %.2e N\n', abs(F_control_with_map(idx_switch)-F_control_with_map(idx_switch-1)));
fprintf('  无映射误差: %.2e N\n\n', abs(F_control_without_map(idx_switch)-F_control_with_map(idx_switch-1)));

fprintf('状态映射保证了控制力的连续性!\n');
fprintf('这对于机器人等物理系统至关重要，避免了冲击载荷。\n');