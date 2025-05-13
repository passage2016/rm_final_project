import socket
import subprocess
import os
import json

import get_state from get_degree

# 创建 socket 对象
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 绑定服务器地址和端口
host = '127.0.0.1'  # 服务器 IP 地址
port = 10086        # 服务器端口号
server_socket.bind((host, port))

# 开始监听
server_socket.listen(5)
subprocess.check_output(['python', "./src/teleop/scripts/grasp.py"], stderr=subprocess.STDOUT, timeout=10)

while True:
    # 接受客户端连接
    client_socket, addr = server_socket.accept()
    print(f"收到客户端连接：{addr}")

    try:
        # 接收客户端发送的信息
        data = client_socket.recv(1024).decode('utf-8')
        print(f"收到客户端消息：{data}")

        request_data = json.loads(data)
        action = request_data.get("action")
        step = request_data.get("step")
        if data == "left":
            script_name = "./src/teleop/scripts/turn_left.py"
        elif data == "right":
            script_name = "./src/teleop/scripts/turn_right.py"
        elif data == "go":
            script_name = "./src/teleop/scripts/go_straight.py"
        else:
            script_name = None

        subprocess.check_output(['source', "./devel/setup.sh"], stderr=subprocess.STDOUT, timeout=10)
        if script_name:
            result = subprocess.check_output(['python', script_name], stderr=subprocess.STDOUT, timeout=10)
        subprocess.check_output(['python', "./src/teleop/scripts/camera.py", step], stderr=subprocess.STDOUT, timeout=10)
        degree = get_degree(int(step))
        response = {
            "status": "success",
            "degree": degree
        }
        client_socket.send(json.dumps(response).encode('utf-8'))
    finally:
        # 关闭客户端连接
        client_socket.close()