import numpy as np
import matplotlib.pyplot as plt

# 設置角度 (常見的單位圓角度)
degrees = np.array([0, 30, 45, 60, 90, 120, 135, 150, 180, 210, 225, 240,
                    270, 300, 315, 330])
radians = np.radians(degrees)  # 轉換為弧度

# 對應的有理化坐標值
coords = {
    0: "(1, 0)", 30: "(√3/2, 1/2)", 45: "(√2/2, √2/2)", 60: "(1/2, √3/2)", 90: "(0, 1)",
    120: "(-1/2, √3/2)", 135: "(-√2/2, √2/2)", 150: "(-√3/2, 1/2)", 180: "(-1, 0)",
    210: "(-√3/2, -1/2)", 225: "(-√2/2, -√2/2)", 240: "(-1/2, -√3/2)", 270: "(0, -1)",
    300: "(1/2, -√3/2)", 315: "(√2/2, -√2/2)", 330: "(√3/2, -1/2)"
}

# 對應的 π 弧度值
radian_labels = {
    0: "0", 30: "π/6", 45: "π/4", 60: "π/3", 90: "π/2",
    120: "2π/3", 135: "3π/4", 150: "5π/6", 180: "π",
    210: "7π/6", 225: "5π/4", 240: "4π/3", 270: "3π/2",
    300: "5π/3", 315: "7π/4", 330: "11π/6"
}

# 計算坐標 (cosθ, sinθ)
x = np.cos(radians)
y = np.sin(radians)

# 繪製單位圓
fig, ax = plt.subplots(figsize=(8, 8))
theta = np.linspace(0, 2 * np.pi, 300)
ax.plot(np.cos(theta), np.sin(theta), 'k')  # 圓周
ax.axhline(0, color='black', linewidth=1)
ax.axvline(0, color='black', linewidth=1)

# 標記圓上的點
ax.scatter(x, y, color='black', zorder=3)
for i, deg in enumerate(degrees):
    ax.text(x[i] * 1.2, y[i] * 1.2, coords[deg], ha='center', va='center', fontsize=10)
    ax.text(x[i] * 0.8, y[i] * 0.8, f"{degrees[i]}°\n{radian_labels[deg]}", ha='center', va='center', fontsize=9)

# 設置坐標範圍
ax.set_xlim(-1.4, 1.4)
ax.set_ylim(-1.4, 1.4)
ax.set_aspect('equal')
ax.set_xticks([])
ax.set_yticks([])
plt.title("Unit Circle with Key Angles and Rationalized Coordinates")
plt.show()
