import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# 这里生成随机点作为示例，实际使用时请替换为你的数据
file_path = './data/pcd.npy'
points = np.load(file_path)
# 创建一个新的图形
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

indices = np.arange(len(points))

# 随机打乱索引
np.random.shuffle(indices)

# 计算要抽取的样本数量（10%）
sample_size = int(len(points) * 0.02)

# 根据索引抽取样本
sampled_data = points[indices[:sample_size]]
# sampled_data = sampled_data[(sampled_data[:, 1] > -0.5)]

# 绘制所有点
ax.scatter(sampled_data[:, 0], sampled_data[:, 1], sampled_data[:, 2], c='blue', marker='o', s=2)

# 设置坐标轴标签
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')

# 设置标题
plt.title('3D Point Cloud')

# 显示图形
plt.show()