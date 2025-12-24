clc; clear; close all;

%% =============================
% 仿真参数
dt = 0.001;
T  = 6;
t  = 0:dt:T;

%% =============================
% 人施加外力
Fh = zeros(size(t));
Fh(t > 1 & t < 5) = 10;

%% =============================
% 占空比切换参数
Ts = 0.01;      % 切换周期 (50 Hz)
d  = 0.3;       % 导纳占空比

%% =============================
% 导纳参数
Ma = 10;
Ba = 20;
Ka = 10;

xa = 0;
va = 0;

Kp_a = 25;
Kd_a = 10;

% 阻抗参数
Ki = 50;
Bi = 25;

%% =============================
% 虚拟动力学系统
Mv = 20;
Bv = 25;
Kv = 30;

%% =============================
% 阻抗接口
Kc = 100;
Bc = 50;

%% =============================
% 真实系统
Mr = 2;
Br = 10;

%% =============================
% 状态初始化
xv = 0; vv = 0;
xr = 0; vr = 0;

%% =============================
% 数据记录
xv_log = zeros(size(t));
xr_log = zeros(size(t));
Feq_log = zeros(size(t));
Fc_log  = zeros(size(t));
mode_log = zeros(size(t)); % 1 adm / 2 imp
last_Fc = 0;

xd = 3*sin(t);
vd = 3*cos(t);
%% =============================
% 仿真主循环
for k = 1:length(t)-1
    
    %% -------- 占空比时间切换 --------
    tau = mod(t(k), Ts);
     % ===== 导纳控制 =====
        a_ref = (Fh(k) - Ba*(va - vd(k)) - Ka*(xa - xd(k)))/Ma;
        va = va + a_ref * dt;
        xa = xa + va * dt;
        xc = xd(k) + xa;
    if tau < d*Ts
        Feq = Kp_a * (xc - xv) + Kd_a * (va - vv);
        mode_log(k) = 1;
    else
        % ===== 阻抗控制 =====
        Feq = Ki*(xd(k) - xv) + Bi*(vd(k) - vv);
        mode_log(k) = 2;
    end
    
    %% -------- 虚拟动力学系统 --------
    Fc = Kc*(xv - xr) + Bc*(vv - vr);
    av = (Feq - Fc - Bv * vv - Kv * xv)/Mv;
    vv = vv + av*dt;
    xv = xv + vv*dt;

    %% -------- 阻抗接口 --------
    Fc = Kc*(xv - xr) + Bc*(vv - vr);

    %% -------- 真实系统 --------
    ar = (Fc - Br*vr)/Mr;
    vr = vr + ar*dt;
    xr = xr + vr*dt;

    %% -------- 记录 --------
    xv_log(k)  = xv;
    xr_log(k)  = xr;
    Feq_log(k) = Feq;
    Fc_log(k)  = Fc;
end

%% =============================
% 绘图
figure;

subplot(4,1,1)
plot(t, xv_log,'LineWidth',1.5); hold on;
plot(t, xr_log,'--','LineWidth',1.5);
plot(t,xd)
ylabel('Position');
legend('Virtual','Real','desire');
grid on;

subplot(4,1,2)
plot(t, Feq_log,'LineWidth',1.2);hold on;
plot(t, Fc_log,'LineWidth',1.2);
ylabel('F_{eq}');
grid on;

subplot(4,1,3)
plot(t, Fc_log,'LineWidth',1.2);
ylabel('F_{coupling}');
grid on;

subplot(4,1,4)
stairs(t, mode_log,'LineWidth',1.2);
ylabel('Mode');
xlabel('Time (s)');
grid on;
