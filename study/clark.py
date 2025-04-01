import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# 時間軸
f = 60  # 頻率 60Hz
T = 1 / f  # 週期
N = 100  # 取樣點數

t = np.linspace(0, T, N)

# 三相正弦波 (假設幅值為 1)
Va = np.cos(2 * np.pi * f * t)
Vb = np.cos(2 * np.pi * f * t - 2*np.pi/3)
Vc = np.cos(2 * np.pi * f * t + 2*np.pi/3)

# Clarke 變換矩陣 (標準型，不考慮縮放因子)
def clarke_transform(va, vb, vc):
    V_alpha = va - 0.5 * vb - 0.5 * vc
    V_beta = (np.sqrt(3)/2) * (vb - vc)
    return V_alpha, V_beta

V_alpha, V_beta = clarke_transform(Va, Vb, Vc)

# 創建動畫
fig, ax = plt.subplots(1, 2, figsize=(12, 6))

# 左圖: 三相電壓
ax[0].set_title("Three-phase system (abc)")
ax[0].set_xlim(0, T)
ax[0].set_ylim(-1.2, 1.2)
ax[0].set_xlabel("Time")
ax[0].set_ylabel("Voltage")
ax[0].grid()

line_a, = ax[0].plot([], [], 'r', label="$V_a$", lw=2)
line_b, = ax[0].plot([], [], 'g', label="$V_b$", lw=2)
line_c, = ax[0].plot([], [], 'b', label="$V_c$", lw=2)
ax[0].legend()

# 右圖: Clarke 變換後的 αβ 平面 (同時用直角坐標 & 極座標表示)
ax[1].set_title("Clarke Transform (αβ frame)")
ax[1].set_xlim(-1.55, 1.55)
ax[1].set_ylim(-1.55, 1.55)
ax[1].set_xlabel(r"$V_{\alpha}$")
ax[1].set_ylabel(r"$V_{\beta}$")
ax[1].grid()
ax[1].axhline(0, color='k', lw=0.5)
ax[1].axvline(0, color='k', lw=0.5)

# 使用 fig.add_axes 創建極座標子圖
# ax2 = fig.add_axes([0.7, 0.7, 0.25, 0.25], projection='polar')  # 增加極座標子圖
# ax2.set_rticks([])  # 隱藏極座標的徑向刻度
# ax2.set_thetamin(0)  # 設定極座標角度範圍
# ax2.set_thetamax(360)
# ax2.set_theta_offset(np.pi / 2)  # 旋轉以匹配直角坐標系

# 初始點與向量
point, = ax[1].plot([], [], 'ro', markersize=8)  # αβ 平面上的點
line_vec_alpha, = ax[1].plot([], [], 'r', lw=2)  # 直角座標系中的 V_alpha 向量
line_vec_beta, = ax[1].plot([], [], 'g', lw=2)  # 直角座標系中的 V_beta 向量

# polar_vec, = ax2.plot([], [], 'b', lw=2)  # 極座標中的 V_total 向量

# 動畫更新函數
def update(frame):
    # 更新三相波形
    line_a.set_data(t[:frame], Va[:frame])
    line_b.set_data(t[:frame], Vb[:frame])
    line_c.set_data(t[:frame], Vc[:frame])

    # 更新 αβ 坐標系
    V_alpha_frame = V_alpha[frame]
    V_beta_frame = V_beta[frame]

    # 直角坐標系中顯示 V_alpha 和 V_beta 向量
    line_vec_alpha.set_data([0, V_alpha_frame], [0, 0])
    line_vec_beta.set_data([0, 0], [0, V_beta_frame])

    # 在 αβ 平面上顯示合成向量
    point.set_data(V_alpha_frame, V_beta_frame)

    # 極座標中的總向量 V_total
    # V_total = np.sqrt(V_alpha_frame**2 + V_beta_frame**2)  # 總向量的模
    # angle_total = np.arctan2(V_beta_frame, V_alpha_frame)  # 合成向量的角度
    # polar_vec.set_data(angle_total, V_total)

    # return line_a, line_b, line_c, point, line_vec_alpha, line_vec_beta, polar_vec
    return line_a, line_b, line_c, point, line_vec_alpha, line_vec_beta

ani = animation.FuncAnimation(fig, update, frames=N, interval=50, blit=True)
plt.show()
