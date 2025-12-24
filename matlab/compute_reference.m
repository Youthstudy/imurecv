%% 4. 参考轨迹生成函数
function x_ref = compute_reference(t, K_d, M_d, D_d, k_e, x0)
    % 计算理想响应
    zeta = D_d/(2*sqrt(M_d*(K_d+k_e)));
    omega_n = sqrt((K_d+k_e)/M_d);
    omega_d = omega_n*sqrt(1-zeta^2);
    
    x_ref = (K_d/(K_d+k_e))*x0*(1 - exp(-zeta*omega_n*t).*(cos(omega_d*t) + ...
            (zeta*omega_n/omega_d)*sin(omega_d*t)));
end