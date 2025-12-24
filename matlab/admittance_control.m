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