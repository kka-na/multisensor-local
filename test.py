import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# 앵커 위치 정의
P1 = np.array([0, 0, 0])
P2 = np.array([5, 0, 0])
P3 = np.array([2, 5, 0])

# 거리 (r1, r2, r3)
r1 = 3.0
r2 = 3.5
r3 = 4.0

# 삼변측량을 통해 추정된 로봇의 위치 (여기서는 예시로 추정 위치 설정)
# 실제 적용에서는 trilateration 알고리즘을 사용하여 계산해야 함
estimated_position = np.array([2.5, 2, 1.5])

# 3D 플롯 설정
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 앵커 위치 표시
ax.scatter(P1[0], P1[1], P1[2], c='r', marker='o', label='P1 (Anchor 1)')
ax.scatter(P2[0], P2[1], P2[2], c='g', marker='o', label='P2 (Anchor 2)')
ax.scatter(P3[0], P3[1], P3[2], c='b', marker='o', label='P3 (Anchor 3)')

# 거리 구 표시
u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi, 100)
x1 = P1[0] + r1 * np.outer(np.cos(u), np.sin(v))
y1 = P1[1] + r1 * np.outer(np.sin(u), np.sin(v))
z1 = P1[2] + r1 * np.outer(np.ones(np.size(u)), np.cos(v))
ax.plot_surface(x1, y1, z1, color='r', alpha=0.1)

x2 = P2[0] + r2 * np.outer(np.cos(u), np.sin(v))
y2 = P2[1] + r2 * np.outer(np.sin(u), np.sin(v))
z2 = P2[2] + r2 * np.outer(np.ones(np.size(u)), np.cos(v))
ax.plot_surface(x2, y2, z2, color='g', alpha=0.1)

x3 = P3[0] + r3 * np.outer(np.cos(u), np.sin(v))
y3 = P3[1] + r3 * np.outer(np.sin(u), np.sin(v))
z3 = P3[2] + r3 * np.outer(np.ones(np.size(u)), np.cos(v))
ax.plot_surface(x3, y3, z3, color='b', alpha=0.1)

# 추정 위치 표시
ax.scatter(estimated_position[0], estimated_position[1], estimated_position[2], c='k', marker='x', label='Estimated Position')

# 앵커와 추정 위치를 연결하는 선 및 거리 표시
for anchor, color, distance, label in zip([P1, P2, P3], ['r', 'g', 'b'], [r1, r2, r3], ['P1', 'P2', 'P3']):
    ax.plot([anchor[0], estimated_position[0]], [anchor[1], estimated_position[1]], [anchor[2], estimated_position[2]], color=color, linestyle='--')
    mid_point = (anchor + estimated_position) / 2
    ax.text(mid_point[0], mid_point[1], mid_point[2], f'd={distance:.1f}', color=color, fontsize=10, backgroundcolor='w')

# 축 레이블 설정
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# 범례 추가
ax.legend()

# 그래프 타이틀
plt.title("3D Trilateration with Distances")

# 그래프 보여주기
plt.show()
