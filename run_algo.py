# ==============================================================================
# 文件名称: run_algo.py
# 功能描述: 卫星任务规划算法的整体调度模块。
#          串联了数据预处理(InputLoader)、数学建模(ModelBuilder)、
#          调用第三方求解器(CBC Solver) 以及结果解析(ScheduleGenerator)的完整流水线。
# ==============================================================================

import subprocess
from load_input import InputLoader
from build_model import ModelBuilder
from generate_schedule import ScheduleGenerator
import os
import time
import logging

def run(data, save_path):
    """
    执行算法全流程：
    
    参数:
        data (dict): 从 HTTP 接口接收到的原始 JSON 请求数据。
        save_path (str): 运行过程中的临时文件存放路径。
        
    返回:
        dict: 最终的规划方案和指标 JSON 数据，或者在异常情况下的错误文本。
        
    执行步骤:
        1. 预处理：解析外部输入并转换为内部数据结构
        2. 建模：构建 MILP(混合整数线性规划) 模型并输出为 model.lp
        3. 求解：跨平台调用 CBC 求解器对模型文件进行求解，并输出 solu.sol
        4. 后处理：读取 solu.sol 解析结果，生成具体的数传、测控与观测方案
    """
    start_algo_time = time.time()  # 记录算法入口起点时刻

    # ---------------------------------------------------------
    # 步骤 1: 数据预处理
    # ---------------------------------------------------------
    logging.info("Input preprocessing...")
    input_data, input_mapping = InputLoader(data).model_input()
    logging.info("Data parsed successfully")

    logging.info("Model building...")
    lp_file = os.path.join(save_path, 'model.lp')
    sol_file = os.path.join(save_path, 'solu.sol')
    ModelBuilder(input_data, input_mapping, lp_file)
    logging.info("Model(MILP) build successfully")

    logging.info("Calling CBC solver...")
    # 确保清理上一轮可能遗留的旧解文件
    if os.path.exists(sol_file):
        os.remove(sol_file)

    # 跨平台自适应查找 CBC 求解器
    import platform
    import shutil
    if platform.system() == 'Windows':
        # Windows: 使用同目录下的 cbc.exe
        cbc_exe = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cbc.exe")
    else:
        # Linux / Docker 容器: 使用系统 PATH 中通过 apt 安装的 cbc
        cbc_exe = shutil.which("cbc") or "cbc"

    try:
        # 执行命令: cbc model.lp solve solu solu.sol
        subprocess.run([cbc_exe, lp_file, "solve", "solu", sol_file], check=True)
    except Exception as e:
        err_log = f"Error calling CBC solver. Is cbc.exe in the folder? Detail: {e}"
        logging.error(err_log)
        return err_log

    if os.path.exists(sol_file):
        logging.info("Model(MILP) solved successfully")
        runtime_ms = int((time.time() - start_algo_time) * 1000)  # 计算全流程运行总毫秒数
        res_data = ScheduleGenerator(sol_file, input_data, input_mapping, data.get('requests', data.get('requestsList', [])), runtime_ms=runtime_ms).output_schedule()
        logging.info("Plan generated successfully")
        # 清理临时文件 (可选)
        # if os.path.exists(sol_file): os.remove(sol_file)
        # if os.path.exists(lp_file): os.remove(lp_file)
        return res_data
    else:
        err_log = "Solving failed! solu.sol was not generated."
        logging.error(err_log)
        return err_log
