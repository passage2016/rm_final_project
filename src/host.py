import socket
import json
import os
import threading

from agent import ask_ai, ask_deepseek
import get_state from get_degree, get_distance

def connect_client(client_ip, step, action, scan)
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    port = 10086        # 服务器端口号
    client_socket.connect((client_ip, port))

    request_data = {
        "action": action
    }
    json_data = json.dumps(request_data)

    client_socket.send(json_data.encode('utf-8'))

    # 接收服务器返回的 JSON 结果
    response_data = client_socket.recv(4096)
    response = json.loads(response_data.decode('utf-8'))
    
    if scan:
        os.system(f"scp spark@{client_ip}:/home/spark/final/pcd/pcd{step}.npy ./data/pcd/pcd{step}.npy")

    # 关闭连接
    client_socket.close()


def restore_to_global_map(local_map, x, y, angle_degrees):
    # 将角度转换为弧度
    angle_radians = math.radians(angle_degrees - 90)

    # 存储转换后的全局坐标
    global_points = []

    for point in local_map:
        local_x, local_y = point
        rotated_x = local_x * math.cos(angle_radians) - local_y * math.sin(angle_radians)
        rotated_y = local_x * math.sin(angle_radians) + local_y * math.cos(angle_radians)

        global_x = x + rotated_x
        global_y = y + rotated_y

        global_points.append((global_x, global_y))

    return global_points


def save_map(filtered_points, filename, x=None, y=None, angle=None):
    max_values = np.max(filtered_points, axis=0)
    min_values = np.min(filtered_points, axis=0)
    step = 0.5

    if x and y and angle != None:
        x_edges = np.arange(0, max_values[0] + step, step)
    else:
        x_edges = np.arange(min_values[0], max_values[0] + step, step)
    y_edges = np.arange(0, max_values[1] + step, step)

    hist, x_edges, y_edges = np.histogram2d(filtered_points[:, 0], filtered_points[:, 1], bins=[x_edges, y_edges])

    hist = np.where(hist > 0, 1, 0)

    hist = expand_ones(hist)
    plt.figure(figsize=(10, 8))
    plt.pcolormesh(x_edges, y_edges, hist.T, cmap='Blues')
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
    return np.array(list(reversed(hist.T.tolist())))


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
        return "left"
    if action == "右转":
        return "right"
    if action == "直行":
        return "go"
    return ""


client_ip1 = '10.31.45.213'
client_ip2 = '10.31.45.37'
task = 7
x, y, angle = 2, 2, 90

for step in range(2000):
    if step < 3:
        action = ""
    else:
        file_path = f'./data/pcd/pcd{step - 1}.npy'
        scan_map = np.load(file_path)
        save_map(global_points, f"./data/robotic/scan_map{step}.png")
        global_points = global_points + restore_to_global_map(scan_map, x, (12 - y), angle)
        global_points = [x_y.split("_") for x_y in list(set([f"{x}_{y}" for (x, y) in global_points]))]
        global_points = [[float(xy[0]), float(xy[1])] for xy in global_points]
        map =  save_map(np.array(global_points),  f"./data/robotic/global_points{step}.png", x, (12 - y), angle)
        if step < 4:
            action = "go"
        elif step < 16:
            action = "right"
        else:
            action = move(1, scan_map, x, y, angle, task, step)

        if action == "left":
            angle = (angle + 30) % 360
        if action == "right":
            angle = (angle + 330) % 360

            angle_rad = math.radians(angle)
            x, y = x + 0.259 * math.cos(angle_rad), y - 0.259 * math.sin(angle_rad)
        if action == "go":
            angle_rad = math.radians(angle)
            pre_distance = get_distance(step - 1)
            current_distance = get_distance(step)
            move_distance = pre_distance - current_distance
            x, y = x + move_distance * math.cos(angle_rad), y - move_distance * math.sin(angle_rad)

        scan_degree = int(get_degree(step))
        angle = angle + ((scan_degree + 15) % 30 - 15)
        

    thread1 = threading.Thread(target=connect_client, args=(client_ip1, step, action, True))
    thread2 = threading.Thread(target=connect_client, args=(client_ip2, step, action, False))
    thread1.start()
    thread2.start()
    thread1.join()
    thread2.join()
    
    