# ==============================================================================
# 文件名称: main.py
# 功能描述: 卫星任务规划系统主服务入口 (纯 DDS 事件驱动节点)
#          常驻星间数据总线，实时监听侦察需求与预警目标引导，
#          触发 MILP 混合整数规划求解，并通过 DDS 总线发布分配方案与单星动作指令。
# ==============================================================================

import os
import sys
import time
import signal
import logging
import threading
from dds_manager import DDSNodeManager

# ------------------------------------------------------------------------------
# 日志配置
# ------------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger("SatellitePlanningNode")

def main():
    work_dir = os.environ.get('WORK_DIR', os.path.dirname(os.path.abspath(__file__)))
    
    # 统一测试域配置支持 (若设置了 DDS_DOMAIN_ID 则强制使用单一域，方便调试)
    unified_domain = None
    if 'DDS_DOMAIN_ID' in os.environ:
        try:
            unified_domain = int(os.environ['DDS_DOMAIN_ID'])
        except ValueError:
            pass

    logger.info("============================================================")
    logger.info("  卫星任务规划系统 (Satellite Mission Planning Node) - 启动")
    logger.info("  通信架构: OMG DDS / Eclipse CycloneDDS (纯星间总线节点)")
    logger.info("  规划算法: 混合整数线性规划 (MILP) + COIN-OR CBC 求解器")
    logger.info(f"  工作目录: {work_dir}")
    logger.info("============================================================")

    # 1. 实例化 DDS 节点管理器
    manager = DDSNodeManager(work_dir=work_dir, unified_domain=unified_domain)
    
    # 2. 注册系统退出信号监听，确保优雅停机
    shutdown_event = threading.Event()

    def signal_handler(signum, frame):
        logger.info(f"收到终止信号 ({signum})，准备停止 DDS 规划服务...")
        manager.stop()
        shutdown_event.set()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 3. 启动 DDS 需求监听
    manager.start()

    logger.info("系统初始化完毕，主线程已进入事件保活循环 (按 Ctrl+C 退出)。")
    try:
        while not shutdown_event.is_set():
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        logger.info("用户请求中断，正在安全退出...")
        manager.stop()

    logger.info("卫星任务规划 DDS 节点已退出。")

if __name__ == "__main__":
    main()
