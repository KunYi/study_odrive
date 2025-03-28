import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

def create_clarke_transformation_plot():
#     font_path = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
#     print(fm.FontProperties(fname=font_path).get_name())
    plt.rcParams['font.family'] = 'Noto Sans CJK JP'
    plt.figure(figsize=(10, 10))
    plt.title('Clarke 變換的幾何表示', fontsize=15)

    # 三相電壓向量（120度間隔）
    Va = np.array([1, 0])  # A相
    Vb = np.array([-0.5, np.sqrt(3)/2])  # B相
    Vc = np.array([-0.5, -np.sqrt(3)/2])  # C相

    # 更精確的向量繪製
    plt.plot([0, Va[0]], [0, Va[1]], color='red', linewidth=2, label='Va (A相)')
    plt.plot([0, Vb[0]], [0, Vb[1]], color='green', linewidth=2, label='Vb (B相)')
    plt.plot([0, Vc[0]], [0, Vc[1]], color='blue', linewidth=2, label='Vc (C相)')

    plt.text(Va[0], Va[1]+0.1, 'Va', color='red', fontsize=10)
    plt.text(Vb[0]-0.15, Vb[1], 'Vb', color='green', fontsize=10)
    plt.text(Vc[0]-0.15, Vc[1], 'Vc', color='blue', fontsize=10)

    # α軸（藍紫色）
    plt.plot([0, 1.2], [0, 0], color='purple', linestyle='--', linewidth=1.5, label='α 軸')
    plt.text(1.2, 0.1, 'α 軸', color='purple', fontsize=12, verticalalignment='center')

    # β軸（橙色）
    plt.plot([0, 0], [0, 1.2], color='orange', linestyle='--', linewidth=1.5, label='β 軸')
    plt.text(0.15, 1.2, 'β 軸', color='orange', fontsize=12, horizontalalignment='center')

    # 箭頭
    plt.arrow(0, 0, Va[0], Va[1], head_width=0.05, head_length=0.05, fc='red', ec='red')
    plt.arrow(0, 0, Vb[0], Vb[1], head_width=0.05, head_length=0.05, fc='green', ec='green')
    plt.arrow(0, 0, Vc[0], Vc[1], head_width=0.05, head_length=0.05, fc='blue', ec='blue')
    plt.arrow(0, 0, 1.2, 0, head_width=0.05, head_length=0.05, fc='purple', ec='purple')
    plt.arrow(0, 1.2, 0, 0, head_width=0.05, head_length=0.05, fc='orange', ec='orange')

    plt.xlabel('α 軸', fontsize=12)
    plt.ylabel('β 軸', fontsize=12)
    plt.xlim(-1.3, 1.3)
    plt.ylim(-1.3, 1.3)
    plt.grid(True, linestyle='--', linewidth=0.5)
    plt.legend()
    plt.axis('equal')

    plt.tight_layout()
    plt.show()

# 執行繪圖
create_clarke_transformation_plot()
