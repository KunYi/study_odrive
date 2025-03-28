import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# 定義角度範圍
theta = np.linspace(0, 2*np.pi, 100)

# 三相電流波形
i_a = np.cos(theta)
i_b = np.cos(theta - 2*np.pi/3)
i_c = np.cos(theta + 2*np.pi/3)

# Clarke 變換
i_alpha = i_a
i_beta = (i_a + 2*i_b) / np.sqrt(3)

# Park 變換
i_d = i_alpha * np.cos(theta) + i_beta * np.sin(theta)
i_q = -i_alpha * np.sin(theta) + i_beta * np.cos(theta)

fig, axs = plt.subplots(3, 2, figsize=(12, 12))

# ---- 三相電流測量 ----
axs[0, 0].plot(theta * 180/np.pi, i_a, 'r', label="$i_a$")
axs[0, 0].plot(theta * 180/np.pi, i_b, 'g', label="$i_b$")
axs[0, 0].plot(theta * 180/np.pi, i_c, 'b', label="$i_c$")
cursor_a, = axs[0, 0].plot([], [], 'ro')
cursor_b, = axs[0, 0].plot([], [], 'go')
cursor_c, = axs[0, 0].plot([], [], 'bo')
axs[0, 0].set_title("Three-phase currents")
axs[0, 0].legend()
axs[0, 0].set_xlabel("Theta (degrees)")
axs[0, 0].grid()

# Clarke 變換向量圖
ax_clarke = axs[1, 1]
ax_clarke.set_title("Clarke Vector Representation")
ax_clarke.set_xlim(-1.2, 1.2)
ax_clarke.set_ylim(-1.2, 1.2)
ax_clarke.grid()
clarke_quiver = ax_clarke.quiver(0, 0, i_alpha[0], i_beta[0], angles='xy', scale_units='xy', scale=1, color='r')
cursor_clarke, = axs[1, 0].plot([], [], 'ro')

# ---- Clarke 變換 ----
axs[1, 0].plot(theta * 180/np.pi, i_alpha, 'r', label=r"$i_{\alpha}$")
axs[1, 0].plot(theta * 180/np.pi, i_beta, 'b', label=r"$i_{\beta}$")
axs[1, 0].set_title("Clarke Transformation")
axs[1, 0].legend()
axs[1, 0].set_xlabel("Theta (degrees)")
axs[1, 0].grid()

# Park 變換向量圖
ax_park = axs[2, 1]
ax_park.set_title("Park Vector Representation")
ax_park.set_xlim(-1.2, 1.2)
ax_park.set_ylim(-1.2, 1.2)
ax_park.grid()
park_quiver = ax_park.quiver(0, 0, i_d[0], i_q[0], angles='xy', scale_units='xy', scale=1, color='b')
cursor_park, = axs[2, 0].plot([], [], 'ro')

# ---- Park 變換 ----
axs[2, 0].plot(theta * 180/np.pi, i_d, 'r', label="$i_d$")
axs[2, 0].plot(theta * 180/np.pi, i_q, 'b', label="$i_q$")
axs[2, 0].set_title("Park Transformation")
axs[2, 0].legend()
axs[2, 0].set_xlabel("Theta (degrees)")
axs[2, 0].grid()

# 更新動畫函數
def update(frame):
    # 更新三相電流游標
    cursor_a.set_data([theta[frame] * 180/np.pi], [i_a[frame]])
    cursor_b.set_data([theta[frame] * 180/np.pi], [i_b[frame]])
    cursor_c.set_data([theta[frame] * 180/np.pi], [i_c[frame]])

    # 更新 Clarke 變換游標
    cursor_clarke.set_data([theta[frame] * 180/np.pi], [i_alpha[frame]])

    # 更新 Park 變換游標
    cursor_park.set_data([theta[frame] * 180/np.pi], [i_d[frame]])

    # 更新向量
    clarke_quiver.set_UVC(i_alpha[frame], i_beta[frame])
    park_quiver.set_UVC(i_d[frame], i_q[frame])

    return cursor_a, cursor_b, cursor_c, cursor_clarke, cursor_park, clarke_quiver, park_quiver

ani = animation.FuncAnimation(fig, update, frames=len(theta), interval=50, blit=True)

plt.tight_layout()
plt.show()
