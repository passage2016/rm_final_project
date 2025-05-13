import os
import sys
import math

import numpy as np
import cv2
from PIL import Image

from scan import get_scan_result, points_to_map, save_map
from agent import ask_ai, ask_deepseek



def restore_to_global_map(local_map, x, y, angle_degrees):
    # 将角度转换为弧度
    angle_radians = math.radians(angle_degrees - 90)

    # 存储转换后的全局坐标
    global_points = []

    for point in local_map:
        # 获取局部坐标
        local_x, local_y = point

        # 计算从局部坐标到全局坐标的转换
        # 首先旋转坐标系（相对于机器人方向）
        rotated_x = local_x * math.cos(angle_radians) - local_y * math.sin(angle_radians)
        rotated_y = local_x * math.sin(angle_radians) + local_y * math.cos(angle_radians)

        # 将旋转后的坐标转换为原始地图坐标系
        global_x = x + rotated_x
        global_y = y + rotated_y

        # 添加到全局点列表
        global_points.append((global_x, global_y))

    return global_points


def find_action(s):
    keywords = ["左转", "右转", "直行"]
    last_indices = {}

    for keyword in keywords:
        last_index = s.rfind(keyword)  # 使用 rfind 方法查找最后一个出现的位置
        if last_index != -1:  # 如果找到该关键词
            last_indices[keyword] = last_index

    if not last_indices:  # 如果没有找到任何关键词
        return "没有找到关键词"
    
    # 找到索引最大的关键词
    last_occurred = max(last_indices, key=lambda k: last_indices[k])
    
    return last_occurred

def get_prompt(scan_map, x, y, angle):
    map_hight = len(scan_map)
    prompt = f"下面是一张迷宫的地图，1代表墙，0代表空地，左上角是(0, 0)点，我目前在({int(x/2)}, {map_hight - int((50-y)/2)})点（从左往右数第{int((x)/2)+1}列，从上往下数第{map_hight - int((50-y)/2)+1}行），右是0度，上是90度，左是180度，下是270度。我目前的朝向是{angle}度，走转度数增加，右转度数减少。我想走出迷宫，也就是说我需要从边框位置值为0的位置出去，请把我导向此处。请告诉我当前应该直行，左转还是右转，不需要告诉我后续动作。"
    for row in scan_map:
        str_row = "".join([str(i) for i in row])
        prompt = f"{prompt}\n{str_row}"
    return prompt

def move(model_id, scan_map, x, y, angle, task, step):
    prompt = get_prompt(scan_map, x, y, angle)
    if model_id == 0:
        response = ask_ai(prompt)
    elif model_id == 1:
        model_name = "deepseek-chat"
        response = ask_deepseek(model_name, prompt)
    else:
        model_name = "deepseek-reasoner"
        response = ask_deepseek(model_name, prompt)
    action = find_action(response)
    with open(f"./data/task{task:03d}/log/step{step:04d}.txt", "w", encoding="utf-8") as file:
        file.write(f"prompt: {prompt}\nresponse: {response}\naction: {action}")
    if action == "左转":
        return x, y, (angle + 90) % 360
    if action == "右转":
        return x, y, (angle + 270) % 360
    if action == "直行":
        angle_rad = math.radians(angle)
        map_hight = len(scan_map)
        print("map hight: ", map_hight)
        print("from: ", map_hight - 25 + int(y/2), int(x/2))
        print("to: ", map_hight - 25 + int((y - math.sin(angle_rad))/2), int((x + math.cos(angle_rad))/2))
        if x + math.cos(angle_rad) < 0 or x + math.cos(angle_rad) >= 80 or y - math.sin(angle_rad) < 0 or y - math.sin(angle_rad) >= 50:
            return x + math.cos(angle_rad), y - math.sin(angle_rad), angle
        if scan_map[map_hight - 25 + int((y - math.sin(angle_rad))/2)][int((x + math.cos(angle_rad))/2)] == 1:
            return x, y, angle
        if scan_map[map_hight - 25 + int((y - 2 * math.sin(angle_rad))/2)][int((x + 2 * math.cos(angle_rad))/2)] == 1:
            return x, y, angle
        return x + math.cos(angle_rad), y - math.sin(angle_rad), angle

    return x, y, angle

if __name__ == "__main__":
    global_points = []
    x, y, angle = 10, 40, 0
    data = np.genfromtxt('map.csv', delimiter=',', dtype=int)
    model_id = int(sys.argv[1])
    task = int(sys.argv[2])
    os.system(f"mkdir -p ./data/task{task:03d}/image/")
    os.system(f"mkdir -p ./data/task{task:03d}/log/")

    for i in range(2000):
        visible_points =get_scan_result(data, x, y, 80, angle)
        save_map(np.array(visible_points), f"./data/task{task:03d}/image/scan_map{i:04d}.png")
        global_points = global_points + restore_to_global_map(visible_points, x, (50 - y), angle)
        global_points = [x_y.split("_") for x_y in list(set([f"{x}_{y}" for (x, y) in global_points]))]
        global_points = [[float(xy[0]), float(xy[1])] for xy in global_points]
        scan_map = save_map(np.array(global_points),  f"./data/task{task:03d}/image/global_points{i:04d}.png", x, (50 - y), angle)
        print(len(scan_map), len(scan_map[0]))
        # map = points_to_map(np.array(global_points))
        # for row in list(map):
        #     print(row)
        if i >= 7:
            x, y, angle = move(model_id, scan_map, x, y, angle, task, i)
        elif i >= 3:
            angle = angle + 90
        print(f"step{i}, location: ", x, y, angle)
        
        if x < 0 or x >= 80 or y < 0 or y >= 50:
            print(f"win with {i} step")
            break


    output_mp4 = f"./data/task{task:03d}/output.mp4"  # 输出的MP4文件名

    # 创建一个VideoWriter对象，用于将图像写入MP4文件
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 使用MP4编码格式
    video_writer = cv2.VideoWriter(output_mp4, fourcc, 6, (2000, 800))  # 448是拼接后的宽度（224 + 224），224是高度


    # 遍历文件夹中的所有PNG文件并处理
    for i in range(2000):  # 假设最多有2000帧
        # 读取第一个图像
        img1_path = f"./data/task{task:03d}/image/scan_map{i:04d}.png"
        if os.path.exists(img1_path) and os.path.getsize(img1_path) > 0:
            img1 = Image.open(img1_path)
            img1_array = cv2.cvtColor(np.array(img1), cv2.COLOR_RGB2BGR)  # 转换为NumPy数组

            img2_path = f"./data/task{task:03d}/image/global_points{i:04d}.png"  # 假设第二个图像的命名规则有所不同
            img2 = Image.open(img2_path)
            img2_array = cv2.cvtColor(np.array(img2), cv2.COLOR_RGB2BGR)  # 转换为NumPy数组

            combined_img = cv2.hconcat([img1_array, img2_array])
            video_writer.write(combined_img)
        else:
            break

    # 释放VideoWriter对象
    video_writer.release()
    print("视频生成完成")
