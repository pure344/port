# ==============================================================================
# 文件名称: build_model.py
# 功能描述: 卫星多任务协同规划数学建模引擎 (MILP 混合整数线性规划)
#          针对星间 DDS 数据架构中的侦察任务 (zc) 与预警任务 (yj) 构建优化求解模型，
#          输出标准 LP 格式文件供 CBC 求解器执行高效求解。
# ==============================================================================

import os
from typing import Dict, Any
from constants import BIG_M, WARNING_PRIORITY_MULTIPLIER

class ModelBuilder:
    def __init__(self, input_data: Dict[str, Any], mapping_data: Dict[str, Any], save_path: str):
        self.model = ''
        self.cons_counter = 0
        self.mission_mode = input_data.get("mission_mode", "zc")
        self.sat_dic = input_data["sat_dic"]
        self.task_list = input_data["task_list"]
        self.tgt_dic = input_data["tgt_dic"]
        self.win_list = input_data["win_list"]
        self.rt_list = input_data["rt_list"]
        self.cluster_info = input_data['cluster_info']
        self.sat_earliest_end = input_data.get('sat_earliest_end', {})
        self.sat_endest_start = input_data.get('sat_endest_start', {})
        self.sat_task = mapping_data["sat_task"]
        self.tgt_task = mapping_data["tgt_task"]
        self.sat_win = mapping_data["sat_win"]
        self.sat_rt = mapping_data["sat_rt"]
        self.save_path = save_path

        # 动态界定时间跨度与 Big-M
        if self.task_list:
            st = min([task['start_time'] for task in self.task_list])
            et = max([task['end_time'] for task in self.task_list])
            time_span = et - st
            self.M = max(time_span * 2.0, BIG_M)
        else:
            self.M = BIG_M

        self.build_objective()
        self.build_constraint()
        self.build_variable()
        self.save_model()

    def build_objective(self):
        """
        构建目标函数：
        - 侦察任务 (zc): 依据目标优先级 pointTargetPriority / areaTargetPriority 加权 (101 - priority)
        - 预警任务 (yj): 叠加 WARNING_PRIORITY_MULTIPLIER，实现绝对高优与连续性保障
        """
        self.model += 'Maximize\nobj:'
        if not self.task_list:
            self.model += ' 0 dummy_var'
            return

        for task in self.task_list:
            t_idx = task['task_idx']
            tgt_id = task['tgt_id']
            tgt_info = self.tgt_dic.get(tgt_id, {})
            
            # priority: 1 ~ 100, 数值越小优先级越高，转换为收益系数
            raw_prio = int(tgt_info.get('priority', 1))
            weight = max(1, 101 - raw_prio) * 100.0

            if self.mission_mode == 'yj' or tgt_info.get('is_emergency'):
                weight = weight * WARNING_PRIORITY_MULTIPLIER

            self.model += f' + {weight} x_{t_idx}'

    def build_constraint(self):
        M = self.M
        self.model += '\nSubject to\n'

        # 1. 目标单次/多度观测约束
        for tgt_id, task_idxs in self.tgt_task.items():
            tgt_info = self.tgt_dic.get(tgt_id, {})
            # 预警动态连续跟踪任务(tracking)属于多星接力跟踪场景，各星负责不同时空弧段接力观测，不设全星座仅限1次的约束
            if self.mission_mode == 'yj' and tgt_info.get('type') == 'tracking':
                continue
            if len(task_idxs) > 1:
                repeat_limit = tgt_info.get("repeat_count", 1)
                self.model += f'tgt_repeat_cons{self.cons_counter}: '
                self.cons_counter += 1
                for item in task_idxs:
                    self.model += f'+ 1 x_{item} '
                self.model += f'<= {repeat_limit}\n'

        # 2. 测控指令开机注入时序约束
        for task in self.task_list:
            sat_id = task['sat_id']
            safe_sat = str(sat_id).replace('-', '_')
            tc_dur = self.sat_dic.get(sat_id, {}).get('tc_duration', 1)
            st = task["start_time"]
            t_idx = task["task_idx"]
            self.model += f'tc_cons_{self.cons_counter}: tc_{safe_sat} + {M} x_{t_idx} <= {st + M - tc_dur}\n'
            self.cons_counter += 1

        # 3. 星地数传与处理时延约束
        for task in self.task_list:
            sat_id = task['sat_id']
            et = task["end_time"]
            dur = task["duration"]
            pr = self.sat_dic.get(sat_id, {}).get("process_rate", 0.5)
            t_idx = task["task_idx"]
            self.model += f's2gt_cons{self.cons_counter}: - 1 s2gt_{t_idx} + {M} x_{t_idx} <= {M - et - dur * pr}\n'
            self.cons_counter += 1

        # 4. 同星任务时间重叠互斥约束 (考虑姿态机动缓冲)
        for sat_id, task_idxs in self.sat_task.items():
            n = len(task_idxs)
            for idx_a in range(n):
                for idx_b in range(idx_a + 1, n):
                    i = task_idxs[idx_a]
                    j = task_idxs[idx_b]
                    task_i = self.task_list[i]
                    task_j = self.task_list[j]
                    
                    st_i, et_i = task_i['start_time'], task_i['end_time']
                    st_j, et_j = task_j['start_time'], task_j['end_time']

                    buffer_time = 5.0
                    if not (et_i + buffer_time <= st_j or et_j + buffer_time <= st_i):
                        self.model += f'overlap_cons{self.cons_counter}: x_{i} + x_{j} <= 1\n'
                        self.cons_counter += 1

    def build_variable(self):
        self.model += 'Bounds\n'
        # 测控上注时间边界
        for sat_id in self.sat_dic:
            safe_sat = str(sat_id).replace('-', '_')
            if self.sat_win.get(sat_id):
                wins = [(self.win_list[i]['start_time'], self.win_list[i]['end_time']) 
                        for i in self.sat_win[sat_id] 
                        if 'HR' in self.win_list[i].get('type', '') or 'Uplink' in self.win_list[i].get('type', '')]
                if wins:
                    tc_min = min(w[0] for w in wins)
                    tc_max = max(w[1] for w in wins)
                    self.model += f'{tc_min} <= tc_{safe_sat} <= {tc_max}\n'

        # 数传时间边界
        for sat_id in self.sat_dic:
            if self.sat_win.get(sat_id):
                dl_wins = [(self.win_list[i]['start_time'], self.win_list[i]['end_time']) 
                           for i in self.sat_win[sat_id]
                           if self.is_downlink(self.win_list[i].get('type', ''))]
                if dl_wins:
                    dl_min = min(w[0] for w in dl_wins)
                    dl_max = max(w[1] for w in dl_wins)
                    for task_idx in self.sat_task.get(sat_id, []):
                        self.model += f'{dl_min} <= s2gt_{task_idx} <= {dl_max}\n'

        self.model += 'Binaries\n'
        for task in self.task_list:
            t_idx = task["task_idx"]
            self.model += f'x_{t_idx}\n'

        self.model += 'End\n'

    def is_downlink(self, win_type: str) -> bool:
        type_list = win_type.split('+')
        for t in type_list:
            if t in ['R', 'HR', 'Downlink']:
                return True
        return False

    def is_tc(self, win_type: str) -> bool:
        type_list = win_type.split('+')
        for t in type_list:
            if t in ['HR', 'Uplink']:
                return True
        return False

    def save_model(self):
        with open(self.save_path, "w", encoding="utf-8") as f:
            f.write(self.model)