import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import lti, step

# ==============================
# 1. 参数设置（完全按论文 Table）
# ==============================
m_true = 1.0      # 真实质量
m_hat = 0.8       # 模型估计质量（用于阻抗控制律）
ke = 3200         # 环境刚度: 尝试 10, 300, 3200
cv = 1.0          # 粘性摩擦系数
Fc = 3.0          # 库仑摩擦

# 期望动态
Md = 1.0
Kd = 100.0
Dd = 2 * 0.7 * np.sqrt(Kd * Md)

# 导纳控制内环 PD（高增益）
kp = 1e6
kv = 2 * 0.7 * np.sqrt(kp * m_true)

# 切换参数
delta = 0.01      # 10 ms 切换周期
n = 0.5           # 占空比: 0=纯阻抗, 1=纯导纳

# 仿真设置
T_end = 1.0
dt_sim = 0.001    # 1ms 仿真步长
x0_ref = 1.0      # 阶跃目标（t>=0 时）

# ==============================
# 2. 摩擦模型
# ==============================
def friction_force(xdot):
    if abs(xdot) < 1e-6:
        return 0.0  # 静摩擦简化处理（实际更复杂）
    else:
        return -np.sign(xdot) * (cv * abs(xdot) + Fc)

# ==============================
# 3. 主仿真（显式欧拉 + 切换）
# ==============================
N = int(T_end / dt_sim) + 1
t_log = np.linspace(0, T_end, N)
x_log = np.zeros(N)
xdot_log = np.zeros(N)
F_log = np.zeros(N)
Fext_log = np.zeros(N)
mode_log = np.empty(N, dtype='<U12')

# 初始状态
x = 0.0
xdot = 0.0
mode = 'impedance'
k_cycle = 0

# 导纳模式额外状态
xd = 0.0
xddot = 0.0

for i in range(N):
    t = t_log[i]
    
    # 当前周期边界
    t_cycle_start = k_cycle * delta
    t_switch_imp_to_adm = t_cycle_start + (1 - n) * delta
    t_cycle_end = (k_cycle + 1) * delta

    # 环境力（弹簧）
    e_pos = x - x0_ref
    F_ext = -ke * e_pos
    
    # 摩擦力（仅作用于真实动力学）
    F_fric = friction_force(xdot)
    
    if mode == 'impedance':
        # ===== 阻抗控制律（使用 m_hat）=====
        e = x - x0_ref
        edot = xdot  # 因为 x0_dot = 0
        F_control = (m_hat / Md - 1) * F_ext + m_hat * 0 - (m_hat / Md) * (Dd * edot + Kd * e)
        
        # 更新模式？
        if t >= t_switch_imp_to_adm and t < T_end:
            # 切换到导纳：计算 xd, xddot 保证 F 连续
            F_imp = F_control
            xd = x + (F_imp + kv * xdot) / kp
            xddot = (F_imp + F_ext + F_fric) / m_true  # 真实加速度
            mode = 'admittance'
    
    elif mode == 'admittance':
        # ===== 导纳控制律 =====
        ed = xd - x0_ref
        # 外环：生成 xd_ddot
        xddot_cmd = (F_ext - Dd * xddot - Kd * ed) / Md
        # 内环 PD 控制力
        F_control = kp * (xd - x) - kv * xdot
        
        # 积分 xd 和 xddot（外环）
        xddot = xddot_cmd  # 这里简化：认为 xddot 跟踪 xddot_cmd（实际有动态，但论文忽略）
        xd += xddot * dt_sim
        
        # 更新模式？
        if t >= t_cycle_end and t < T_end:
            mode = 'impedance'
            k_cycle += 1

    # === 真实系统动力学（总是用 m_true）===
    xddot_actual = (F_control + F_ext + F_fric) / m_true
    xdot += xddot_actual * dt_sim
    x += xdot * dt_sim

    # === 记录 ===
    x_log[i] = x
    xdot_log[i] = xdot
    F_log[i] = F_control
    Fext_log[i] = F_ext
    mode_log[i] = mode

# ==============================
# 4. 理想响应（无摩擦、无建模误差）
# ==============================
# 理想闭环：Md*e_ddot + Dd*e_dot + (Kd + ke)*e = 0, e = x - x0
num = [Kd]
den = [Md, Dd, Kd + ke]
sys_ideal = lti(num, den)
t_ideal, x_ideal = step(sys_ideal, T=np.linspace(0, T_end, 1000))
x_ideal *= x0_ref  # 阶跃幅值

# ==============================
# 5. 绘图
# ==============================
plt.figure(figsize=(12, 10))

# (a) 位置
plt.subplot(4, 1, 1)
plt.plot(t_ideal, x_ideal, 'k--', linewidth=1.5, label='Ideal Response')
plt.plot(t_log, x_log, 'b', linewidth=1.2, label='Hybrid Controller')
plt.ylabel('Position $x(t)$ [m]')
plt.legend(loc='lower right')
plt.grid(True)

# (b) 控制器输出力 F
plt.subplot(4, 1, 2)
plt.plot(t_log, F_log, 'r', linewidth=1.2, label='Control Force $F$')
plt.ylabel('Force $F$ [N]')
plt.legend()
plt.grid(True)

# (c) 环境交互力 F_ext
plt.subplot(4, 1, 3)
plt.plot(t_log, Fext_log, 'g', linewidth=1.2, label='Environment Force $F_{ext}$')
plt.ylabel('Force $F_{ext}$ [N]')
plt.legend()
plt.grid(True)

# (d) 控制模式
plt.subplot(4, 1, 4)
mode_int = np.where(mode_log == 'admittance', 1, 0)
plt.step(t_log, mode_int, where='post', linewidth=1.2)
plt.yticks([0, 1], ['Impedance', 'Admittance'])
plt.ylabel('Control Mode')
plt.xlabel('Time [s]')
plt.grid(True)

plt.suptitle(f'Hybrid Impedance/Admittance Control (ke = {ke} N/m, with Friction & Model Error)', fontsize=14)
plt.tight_layout(rect=[0, 0, 1, 0.97])
plt.show()

# ==============================
# 6. 打印最终稳态误差（可选）
# ==============================
steady_error = abs(x_log[-1] - x_ideal[-1])
print(f"Final position: {x_log[-1]:.4f} m")
print(f"Ideal final position: {x_ideal[-1]:.4f} m")
print(f"Steady-state error: {steady_error:.4f} m")