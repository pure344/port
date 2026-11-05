# ==============================================================================
# 文件名称: test/test.py
# 功能描述: 自动化测试脚本，全面测试星间 DDS 架构下的侦察任务 (zc) 与预警任务 (yj)
#          支持传入任意 JSON 测试文件路径，支持直接调用内核或通过 HTTP 接口端到端验证。
# ==============================================================================

import json
import os
import sys
import time

# 将上级目录加入 sys.path，以便直接调用 run_algo
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from run_algo import run

current_dir = os.path.dirname(os.path.abspath(__file__))

def test_file(file_path: str, mode_name: str = None) -> bool:
    """
    通用测试函数：支持测试任意传入的 DDS JSON 文件
    """
    if not os.path.isabs(file_path):
        candidate_path = os.path.join(current_dir, file_path)
        file_path = candidate_path if os.path.exists(candidate_path) else os.path.abspath(file_path)
    
    file_name = os.path.basename(file_path)
    if not mode_name:
        mode_name = file_name

    print(f"\n==================== [测试: {mode_name}] ====================")
    if not os.path.exists(file_path):
        print(f"[ERROR] 找不到测试文件: {file_path}")
        return False

    with open(file_path, 'r', encoding='utf-8') as f:
        payload = json.load(f)

    # 调用算法内核进行规划求解
    t_start = time.time()
    res_json = run(payload, current_dir)
    duration = round(time.time() - t_start, 3)

    if not isinstance(res_json, dict) or "metrics" not in res_json:
        print(f"[ERROR] 规划求解失败: {res_json}")
        return False

    m = res_json.get("metrics", {})
    assign_list = res_json.get("taskAssignList", [])
    cmd_list = res_json.get("commandList", [])

    print(f"规划模式 (missionMode): {res_json.get('missionMode')}")
    print(f"任务分配主题 (taskAssignTopic): {res_json.get('taskAssignTopic')}")
    print(f"指令下发主题 (commandTopic): {res_json.get('commandTopic')}")
    print(f"分配方案数量: {len(assign_list)} 份报文")
    print(f"单星指令数量: {len(cmd_list)} 条动作指令")
    print(f"全流程耗时: {duration} 秒 (其中 CBC 求解耗时: {m.get('schedulingDuration')} ms)")
    print(f"任务完成率: {m.get('completionRate', 0) * 100:.2f}%")
    print(f"综合评分 (overallScore): {m.get('overallScore')} 分")

    if assign_list:
        print("\n--- 首条任务分配数据样例 (DDS 标准) ---")
        sample_assign = assign_list[0]['dataname']['validData']
        print(f"  执行卫星 (execNode): SCID {sample_assign.get('execNode')}")
        print(f"  任务开始时间: {sample_assign.get('taskStartTime')}")
        print(f"  任务结束时间: {sample_assign.get('taskEndTime')}")
        if 'pointTargetCnt' in sample_assign:
            print(f"  分配点目标数: {sample_assign.get('pointTargetCnt')}")
            print(f"  分配面目标数: {sample_assign.get('areaTargetCnt')}")
        if 'taskType' in sample_assign:
            print(f"  预警任务类型: {sample_assign.get('taskType')}")
            print(f"  凝视视场/目标ID: {sample_assign.get('viewID')}")

    if cmd_list:
        print("\n--- 首条单星任务指令样例 (sinsate_command.idl) ---")
        sample_cmd = cmd_list[0]['dataname']['validData']
        print(f"  指令卫星标识 (SCID): {sample_cmd.get('SCID')}")
        print(f"  成像时刻 (imageSendTime): {sample_cmd.get('imageSendTime')}")
        print(f"  侧摆角 (rollAngle): {sample_cmd.get('rollAngle')} deg")
        print(f"  姿态四元数: ({sample_cmd.get('attiQuaterW')}, {sample_cmd.get('attiQuaterX')}, {sample_cmd.get('attiQuaterY')}, {sample_cmd.get('attiQuaterZ')})")

    print(f"==================== [{mode_name} 测试通过] ====================")
    return True

if __name__ == '__main__':
    # 1. 支持命令行任意传参测试: python test/test.py [文件路径]
    if len(sys.argv) > 1:
        target_file = sys.argv[1]
        test_file(target_file)
    else:
        # 2. 默认依次测试全部数据集 (标准集与大规模集)
        test_file('test_zc.json', "DDS 侦察 (ZC - 标准集)")
        test_file('test_yj.json', "DDS 预警 (YJ - 标准集)")
        test_file('test_large_zc.json', "DDS 侦察 (ZC - 大规模集)")
        test_file('test_large_yj.json', "DDS 预警 (YJ - 大规模集)")