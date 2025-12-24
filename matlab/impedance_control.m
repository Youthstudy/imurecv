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