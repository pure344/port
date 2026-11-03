# ==============================================================================
# 文件名称: run_algo.py
# 功能描述: 卫星任务规划算法全流程执行与调度模块 (DDS 架构适配版)
# ==============================================================================

import subprocess
import os
import time
import logging
import platform
import shutil
from load_input import InputLoader
from build_model import ModelBuilder
from generate_schedule import ScheduleGenerator

def run(data, save_path):
    """
    执行算法全流程：
    
    参数:
        data (dict): 从 HTTP 接口接收到的 DDS 格式 JSON 请求数据。
        save_path (str): 运行过程中的临时文件存放路径。
        
    返回:
        dict: 最终的 DDS 规划方案和指标 JSON 数据。
    """
    start_algo_time = time.time()
    os.makedirs(save_path, exist_ok=True)

    # 1. 预处理 (DDS 架构解析)
    logging.info("Input preprocessing with DDS schema...")
    input_data, input_mapping = InputLoader(data).model_input()
    logging.info(f"DDS Data parsed successfully. Mission Mode: {input_data.get('mission_mode')}")

    # 2. 建模 (MILP LP 生成)
    logging.info("Model building...")
    lp_file = os.path.join(save_path, 'model.lp')
    sol_file = os.path.join(save_path, 'solu.sol')
    ModelBuilder(input_data, input_mapping, lp_file)
    logging.info("Model(MILP) built successfully")

    # 3. 求解 (CBC Solver)
    logging.info("Calling CBC solver...")
    if os.path.exists(sol_file):
        os.remove(sol_file)

    if platform.system() == 'Windows':
        cbc_exe = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cbc.exe")
    else:
        cbc_exe = shutil.which("cbc") or "cbc"

    try:
        subprocess.run([cbc_exe, lp_file, "solve", "solu", sol_file], check=True)
    except Exception as e:
        err_log = f"Error calling CBC solver. Is cbc.exe available? Detail: {e}"
        logging.error(err_log)
        return {"error": err_log, "status": 500}

    # 4. 后处理 (生成 DDS 规范分配数据与单星指令)
    if os.path.exists(sol_file):
        logging.info("Model(MILP) solved successfully")
        runtime_ms = int((time.time() - start_algo_time) * 1000)
        res_data = ScheduleGenerator(
            sol_file,
            input_data,
            input_mapping,
            runtime_ms=runtime_ms
        ).output_schedule()
        logging.info("DDS Plan generated successfully")
        return res_data
    else:
        err_log = "Solving failed! solu.sol was not generated."
        logging.error(err_log)
        return {"error": err_log, "status": 500}
