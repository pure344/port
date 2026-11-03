# ==============================================================================
# 文件名称: main.py
# 功能描述: 卫星任务规划系统的 HTTP 服务主入口。
#          负责监听指定端口，接收外部的 JSON 格式任务需求，
#          调用核心规划算法进行求解，并将结果实时返回给客户端。
# ==============================================================================

from http.server import HTTPServer, BaseHTTPRequestHandler
import json
import logging
import os
import traceback
from http.server import HTTPServer, BaseHTTPRequestHandler
from run_algo import run

# ------------------------------------------------------------------------------
# 全局日志与环境变量配置
# ------------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - [%(levelname)s] - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

# APP_PORT: 服务监听的端口号，默认 8080，适配 Docker 容器环境
port = int(os.environ.get('APP_PORT', 8080))
# WORK_DIR: 工作目录，用于存放过程临时文件，默认当前代码所在目录
path = os.environ.get('WORK_DIR', os.path.dirname(os.path.abspath(__file__)))
# RUN_MODE: 运行模式，为了兼容历史逻辑保留，现已全面转为 server 模式
local = os.environ.get('RUN_MODE', 'server') == 'local'

class PostHandler(BaseHTTPRequestHandler):
    """
    HTTP 请求处理类，继承自 BaseHTTPRequestHandler。
    用于处理客户端发送的 POST 规划请求以及 GET 健康检查请求。
    """
    
    def do_POST(self):
        """
        处理 POST 请求：接收卫星规划任务参数，并调用核心算法求解。
        
        流程:
        1. 读取 HTTP 请求体中的 JSON 字节流。
        2. 将输入数据落盘保存为 chenhao.json（用于问题排查与回溯）。
        3. 调用 run_algo 模块的 run 函数进行核心规划求解。
        4. 将求解结果落盘保存为 chenhao_res.json。
        5. 将求解结果以 JSON 格式作为 HTTP 响应返回。
        """
        try:
            # 获取请求体长度
            length = int(self.headers['Content-Length'])
            # 读取并解析 JSON 数据
            data = json.loads(self.rfile.read(length))
            
            logging.info("Received planning request! Start planning...")
            res_data = run(data, path)
            
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            # 增加 indent=4 让 HTTP 接口返回的 JSON 同样具备高可读性的规整格式
            self.wfile.write(json.dumps(res_data, ensure_ascii=False, indent=4).encode('utf-8'))
            logging.info("Planning response sent successfully.")
            
        except ValueError as ve:
            # 专门捕获由于输入数据异常（如格式不合法、必填字段缺失）引发的友好报错
            err_msg = str(ve)
            logging.warning(f"Data Validation Error: {err_msg}")
            self.send_response(400) # Bad Request
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"error": err_msg, "status": 400}, ensure_ascii=False).encode('utf-8'))
            
        except Exception as e:
            err_msg = traceback.format_exc()
            logging.error(f"Server Internal Error:\n{err_msg}")
            self.send_response(500) # Internal Server Error
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"error": str(e), "traceback": err_msg, "status": 500}, ensure_ascii=False).encode('utf-8'))

    def do_GET(self):
        """处理 GET 请求：健康检查接口"""
        if self.path == '/health/live':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "UP"}).encode('utf-8'))
        else:
            self.send_response(404)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"error": "Not Found"}).encode('utf-8'))

if __name__ == "__main__":
    server = HTTPServer(('0.0.0.0', port), PostHandler)
    print(f"Satellite Planning Service listening on port {port}")
    server.serve_forever()