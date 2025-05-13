import numpy as np
import math
import matplotlib.pyplot as plt
from scipy import signal

def expand_ones(image):
    # 创建一个 3x3 的全 1 内核
    kernel = np.ones((2, 2))
    # 使用卷积操作，这里使用 'same' 模式来保持输出图像的大小不变
    expanded = signal.convolve2d(image, kernel, mode='same')
    # 将所有大于 0 的值设置为 1，其余设置为 0
    expanded = (expanded > 0).astype(np.uint8)
    return expanded

def get_visible_map(maze, x, y, angle_range, angle_degrees):
    # 迷宫形状
    height, width = maze.shape

    angle_range = math.radians(angle_range)
    angle_start = math.radians(angle_degrees) - angle_range / 2
    angle_end = math.radians(angle_degrees) + angle_range / 2

    visible_map = np.zeros_like(maze)

    # 定义可视范围的射线数量
    num_rays = 1000
    angles = np.linspace(angle_start, angle_end, num_rays)

    max_step = max(height, width) * 2

    for ray_angle in angles:
        step_x = math.cos(ray_angle)
        step_y = math.sin(ray_angle)

        current_x, current_y = x, y
        for _ in range(max_step):
            current_x += step_x
            current_y += step_y

            if 0 <= current_x < width and 0 <= current_y < height:
                if maze[int(current_y), int(current_x)] == 1:  # 遇到墙
                    visible_map[int(current_y), int(current_x)] = 1  # 可见区域
                    break
            else:
                break

    return visible_map

def points_to_map(filtered_points):
    max_values = np.max(filtered_points, axis=0)
    min_values = np.min(filtered_points, axis=0)
    # print(max_values)
    # print(min_values)

    step = 2

    # 定义每个维度的边界点
    x_edges = np.arange(min_values[0], max_values[0] + step, step)
    y_edges = np.arange(0, max_values[1] + step, step)

    # 统计每个小格点的数量
    hist, x_edges, y_edges = np.histogram2d(filtered_points[:, 0], filtered_points[:, 1], bins=[x_edges, y_edges])

    # 打印结果
    # print("每个小格点的计数:")
    hist = np.where(hist > 0, 1, 0)

    hist = expand_ones(hist)
    # for row in list(reversed(hist.T.tolist())):
    #     print(row)
    # 可视化结果
    plt.figure(figsize=(10, 8))
    plt.pcolormesh(x_edges, y_edges, hist.T, cmap='Blues')
    plt.colorbar(label='计数')
    # plt.title('每个小格点的数量')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()
    return np.array(list(reversed(hist.T.tolist())))

def save_map(filtered_points, filename, x=None, y=None, angle=None):
    max_values = np.max(filtered_points, axis=0)
    min_values = np.min(filtered_points, axis=0)
    step = 2

    if x and y and angle != None:
        x_edges = np.arange(0, max_values[0] + step, step)
    else:
        x_edges = np.arange(min_values[0], max_values[0] + step, step)
    y_edges = np.arange(0, max_values[1] + step, step)

    hist, x_edges, y_edges = np.histogram2d(filtered_points[:, 0], filtered_points[:, 1], bins=[x_edges, y_edges])

    hist = np.where(hist > 0, 1, 0)

    # hist = expand_ones(hist)
    plt.figure(figsize=(10, 8))
    plt.pcolormesh(x_edges, y_edges, hist.T, cmap='Blues')
    # plt.colorbar(label='计数')
    # plt.title('每个小格点的数量')
    plt.xlabel('X')
    plt.ylabel('Y')
    if x and y and angle != None:
        # 添加红点
        plt.scatter([x], [y], color='red', s=10)  # s控制点的大小

        # 添加红色线段
        angle_rad = math.radians(angle+40)
        x_values = [x, x + math.cos(angle_rad)]
        y_values = [y, y + math.sin(angle_rad)]
        plt.plot(x_values, y_values, color='red', linewidth=2)

        angle_rad = math.radians(angle - 40)
        x_values = [x, x + math.cos(angle_rad)]
        y_values = [y, y + math.sin(angle_rad)]
        plt.plot(x_values, y_values, color='red', linewidth=2)

    plt.savefig(filename)
    plt.close()
    return list(reversed(hist.T.tolist()))

def get_scan_result(maze, x, y, angle_range, angle_degrees):
    # 迷宫形状
    height, width = maze.shape
    angle_degrees = -angle_degrees

    angle_range = math.radians(angle_range)
    angle_start = math.radians(angle_degrees) - angle_range / 2
    angle_end = math.radians(angle_degrees) + angle_range / 2

    visible_points = []

    # 定义可视范围的射线数量
    num_rays = 10000
    angles = np.linspace(angle_start, angle_end, num_rays)

    max_step = max(height, width) * 2

    for ray_angle in angles:
        step_x = math.cos(ray_angle)
        step_y = math.sin(ray_angle)

        current_x, current_y = x, y
        for _ in range(max_step):
            current_x += step_x
            current_y += step_y

            if 0 <= current_x < width and 0 <= current_y < height:
                if maze[int(current_y), int(current_x)] == 1:  # 遇到墙
                    distance = math.sqrt((current_y - y) * (current_y - y) + (current_x - x) * (current_x - x))
                    trans_ray_angle = ray_angle + math.radians(90) - math.radians(angle_degrees)
                    # print(ray_angle)
                    # print(trans_ray_angle)
                    # print(distance)
                    # print(int(current_y), int(current_x))
                    trans_x = - distance * math.cos(trans_ray_angle)
                    trans_y = distance * math.sin(trans_ray_angle)
                    visible_points.append([trans_x, trans_y])
                    break
            else:
                break

    return visible_points


def scan(map_file, x, y, angle, angle_range = 80):
    data = np.genfromtxt(map_file, delimiter=',', dtype=int)

    visible_map = get_scan_result(data, x, y, angle_range, angle)

    points = np.array(visible_map)
    # print(np.max(points, axis=0))
    # print(np.min(points, axis=0))

    points_to_map(points)
    return points

# scan('map.csv', 10, 40, 0)