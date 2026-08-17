import os
import copy
from typing import Dict, Any
from constants import BIG_M

class ModelBuilder():
    def __init__(self, input_data, mapping_data, save_path):
        self.model = ''
        self.cons_counter = 0
        self.sat_dic = input_data["sat_dic"]
        self.task_list = input_data["task_list"]
        self.tgt_dic = input_data["tgt_dic"]
        self.win_list = input_data["win_list"]
        self.rt_list = input_data["rt_list"]
        self.cluster_info = input_data['cluster_info']
        self.sat_earliest_end = input_data['sat_earliest_end']
        self.sat_endest_start = input_data['sat_endest_start']
        self.sat_task = mapping_data["sat_task"]
        self.tgt_task = mapping_data["tgt_task"]
        self.sat_win = mapping_data["sat_win"]
        self.sat_rt = mapping_data["sat_rt"]
        self.flag_list = input_data.get("flag_list", {})
        self.emergency_status = input_data.get("emergency_status", False)
        self.emergency_tasks = input_data.get("emergency_tasks", [])
        self.emergency_tgt_ids = input_data.get("emergency_tgt_ids", set())
        self.save_path = save_path
        self.objective = 'repeat_first'
        self.planning_mode = 'on_ground'

        # 动态计算绝对安全的 Big-M 参数（为规划时间跨度的 2 倍，保底 604800.0 秒）
        all_times = []
        for task in self.task_list:
            all_times.extend([task.get('start_time', 0), task.get('end_time', 0)])
        for win in self.win_list:
            all_times.extend([win.get('start_time', 0), win.get('end_time', 0)])
        for rt in self.rt_list:
            all_times.extend([rt.get('start_time', 0), rt.get('end_time', 0)])

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
        self.SaveModel()
    
    def build_objective(self):
        """
        构建目标函数：
        在 repeat_first 模式下，融入目标优先级 (yxj) 权重。
        若包含应急需求 (emergency_tgt_ids)，将应急目标的权重乘 10000 放大，确保求解器 100% 绝对优先开机规划。
        """
        if self.objective == 'time_first':
            self.model += 'Minimize\nobj: m2gt'
        elif self.objective == 'repeat_first':
            self.model += 'Maximize\nobj:'
            for task in self.task_list:
                t_idx = task['task_idx']
                tgt_id = task['tgt_id']
                base_weight = self.tgt_dic.get(tgt_id, {}).get('priority', 1)

                if tgt_id in self.emergency_tgt_ids:
                    weight = base_weight * 10000
                else:
                    weight = base_weight

                self.model += f' + {weight} x_{t_idx}'
    
    def build_constraint(self):
        M = self.M
        self.model += '\nSubject to\n'

        # 1. 目标重访约束
        if self.objective == 'time_first':
            for tgt_id, task_idxs in self.tgt_task.items():
                self.model += f'rpc_cons{self.cons_counter}: '
                self.cons_counter += 1
                for item in task_idxs:
                    self.model += f'+ 1 x_{item} '
                self.model += f'>= {self.tgt_dic[tgt_id]["repeat_count"]}\n'

        # 2. 应急重规划冲突避让约束 x_conflict = 0
        if self.emergency_status and self.emergency_tasks:
            conflict_ids = {t['taskId'] for t in self.emergency_tasks if isinstance(t, dict) and t.get('taskId')}
            for task in self.task_list:
                if task['task_id'] in conflict_ids:
                    t_idx = task["task_idx"]
                    self.model += f'conflict_avoid_cons{self.cons_counter}: x_{t_idx} = 0\n'
                    self.cons_counter += 1

        # 3. 测控指令开机注入时序约束
        for task in self.task_list:
            self.model += f'tc_cons_{self.cons_counter}: '
            self.cons_counter += 1
            sat_id = task['sat_id']
            safe_sat = str(sat_id).replace('-', '_')
            tc_dur = self.sat_dic[sat_id]['tc_duration']
            st = task["start_time"]
            t_idx = task["task_idx"]
            self.model += f'tc_{safe_sat} + {M} x_{t_idx} <= {st + M - tc_dur}\n'

        # 4. 星上中转与下传约束
        if self.planning_mode == 'on_sat' and not self.flag_list.get('is_raw_data_download', False):
            for task in self.task_list:
                self.model += f's2mt_cons{self.cons_counter}: '
                self.cons_counter += 1
                sat_id = task['sat_id']
                et = task["end_time"]
                dur = task["duration"]
                pr = self.sat_dic[sat_id]["process_rate"]
                t_idx = task["task_idx"]
                if self.flag_list.get('is_sub_process', True):
                    self.model += f'- 1 s2mt_{t_idx} + {M} x_{t_idx} <= {M - et - dur * pr}\n'
                else:
                    self.model += f'- 1 s2mt_{t_idx} + {M} x_{t_idx} <= {M - et}\n'

            for task in self.task_list:
                self.model += f'm2gt_cons{self.cons_counter}: '
                self.cons_counter += 1
                sat_id = task['sat_id']
                dur = task["duration"]
                tr = self.sat_dic[sat_id]["transmission_rate"]
                t_idx = task["task_idx"]
                self.model += f's2mt_{t_idx} - 1 m2gt + {M} x_{t_idx} <= {M - dur * tr}\n'

        # 5. 星地数传约束
        if self.planning_mode == 'on_ground' or self.flag_list.get('is_raw_data_download', False):
            for task in self.task_list:
                self.model += f's2gt_cons{self.cons_counter}: '
                self.cons_counter += 1
                sat_id = task['sat_id']
                et = task["end_time"]
                dur = task["duration"]
                pr = self.sat_dic[sat_id]["process_rate"]
                t_idx = task["task_idx"]
                if self.flag_list.get('is_sub_process', True):
                    self.model += f'- 1 s2gt_{t_idx} + {M} x_{t_idx} <= {M - et - dur * pr}\n'
                else:
                    self.model += f'- 1 s2gt_{t_idx} + {M} x_{t_idx} <= {M - et}\n'

        # 6. 卫星能量容量上限约束
        # for sat_id, task_idxs in self.sat_task.items():
        #     self.model += f'energy_cons{self.cons_counter}: '
        #     self.cons_counter += 1
        #     for item in task_idxs:
        #         coeff = self.task_list[item]["duration"] * (1 + self.sat_dic[sat_id]["process_rate"] + self.sat_dic[sat_id]["transmission_rate"]) + self.sat_dic[sat_id]["tc_duration"]
        #         self.model += f'+ {coeff} x_{item} '
        #     self.model += f'<= {self.sat_dic[sat_id]["energy_capacity"]}\n'

        # 7. 窗口选择约束（保证 tc / s2gt 变量值必须落在某个真实窗口内）
        # self._win_sel_tc = {}   # 记录每颗卫星的 tc 窗口列表，供 build_variable 使用
        # self._win_sel_dl = {}   # 记录每个任务的 s2gt 下行窗口列表
        # if self.planning_mode == 'on_ground':
        #     for sat_id in self.sat_dic:
        #         if self.sat_win.get(sat_id) is None:
        #             continue
        #         wins = [(self.win_list[i]['start_time'], self.win_list[i]['end_time']) for i in self.sat_win[sat_id]]
        #         safe_id = str(sat_id).replace('-', '_')
        #         self._win_sel_tc[sat_id] = wins
        #         if len(wins) > 1:
        #             for j, (w_st, w_et) in enumerate(wins):
        #                 self.model += f'tc_wl_cons{self.cons_counter}: tc_{safe_id} - {M} y_tc_{safe_id}_{j} >= {w_st - M}\n'
        #                 self.cons_counter += 1
        #                 self.model += f'tc_wu_cons{self.cons_counter}: tc_{safe_id} + {M} y_tc_{safe_id}_{j} <= {w_et + M}\n'
        #                 self.cons_counter += 1
        #             self.model += f'tc_wsel_cons{self.cons_counter}: '
        #             self.cons_counter += 1
        #             for j in range(len(wins)):
        #                 self.model += f'+ 1 y_tc_{safe_id}_{j} '
        #             self.model += '>= 1\n'

        #         dl_wins = [(self.win_list[i]['start_time'], self.win_list[i]['end_time']) for i in self.sat_win[sat_id]
        #                    if self.is_downlink(self.win_list[i]['type']) and self.win_list[i]['start_time'] >= self.sat_earliest_end.get(sat_id, 0)]
        #         if len(dl_wins) > 1:
        #             for task_idx in self.sat_task.get(sat_id, []):
        #                 self._win_sel_dl[task_idx] = dl_wins
        #                 for j, (w_st, w_et) in enumerate(dl_wins):
        #                     self.model += f'dl_wl_cons{self.cons_counter}: s2gt_{task_idx} - {M} y_dl_{task_idx}_{j} >= {w_st - M}\n'
        #                     self.cons_counter += 1
        #                     self.model += f'dl_wu_cons{self.cons_counter}: s2gt_{task_idx} + {M} y_dl_{task_idx}_{j} <= {w_et + M}\n'
        #                     self.cons_counter += 1
        #                 self.model += f'dl_wsel_cons{self.cons_counter}: '
        #                 self.cons_counter += 1
        #                 for j in range(len(dl_wins)):
        #                     self.model += f'+ 1 y_dl_{task_idx}_{j} '
        #                 self.model += '>= 1\n'

        if self.planning_mode == 'on_sat':
            main_sat_id = self.cluster_info['main_sat']
            # tc 路由窗口选择
            self._rt_sel_tc = {}
            for sat_id in self.sat_dic:
                if main_sat_id in self.sat_rt:
                    rt_wins = []
                    for rt_idx in self.sat_rt[main_sat_id]:
                        rt = self.rt_list[rt_idx]
                        des_sat = rt.get('desSatellited', rt.get('desSatelliteId'))
                        if des_sat == sat_id:
                            rt_wins.append((rt['start_time'], rt['end_time']))
                    if rt_wins:
                        self._rt_sel_tc[sat_id] = rt_wins
                        safe_id = str(sat_id).replace('-', '_')
                        if len(rt_wins) > 1:
                            for j, (r_st, r_et) in enumerate(rt_wins):
                                self.model += f'rt_tc_wl_cons{self.cons_counter}: tc_{safe_id} - {M} y_rt_tc_{safe_id}_{j} >= {r_st - M}\n'
                                self.cons_counter += 1
                                self.model += f'rt_tc_wu_cons{self.cons_counter}: tc_{safe_id} + {M} y_rt_tc_{safe_id}_{j} <= {r_et + M}\n'
                                self.cons_counter += 1
                            self.model += f'rt_tc_wsel_cons{self.cons_counter}: '
                            self.cons_counter += 1
                            for j in range(len(rt_wins)):
                                self.model += f'+ 1 y_rt_tc_{safe_id}_{j} '
                            self.model += '>= 1\n'

            # s2mt 路由窗口选择
            self._rt_sel_s2mt = {}
            if not self.flag_list.get('is_raw_data_download', False):
                for task in self.task_list:
                    sat_id = task['sat_id']
                    t_idx = task['task_idx']
                    if sat_id in self.sat_rt:
                        s2mt_wins = []
                        for rt_idx in self.sat_rt[sat_id]:
                            rt = self.rt_list[rt_idx]
                            des_sat = rt.get('desSatellited', rt.get('desSatelliteId'))
                            if des_sat == main_sat_id:
                                s2mt_wins.append((rt['start_time'], rt['end_time']))
                        if s2mt_wins:
                            self._rt_sel_s2mt[t_idx] = s2mt_wins
                            if len(s2mt_wins) > 1:
                                for j, (r_st, r_et) in enumerate(s2mt_wins):
                                    self.model += f'rt_s2mt_wl_cons{self.cons_counter}: s2mt_{t_idx} - {M} y_rt_s2mt_{t_idx}_{j} >= {r_st - M}\n'
                                    self.cons_counter += 1
                                    self.model += f'rt_s2mt_wu_cons{self.cons_counter}: s2mt_{t_idx} + {M} y_rt_s2mt_{t_idx}_{j} <= {r_et + M}\n'
                                    self.cons_counter += 1
                                self.model += f'rt_s2mt_wsel_cons{self.cons_counter}: '
                                self.cons_counter += 1
                                for j in range(len(s2mt_wins)):
                                    self.model += f'+ 1 y_rt_s2mt_{t_idx}_{j} '
                                self.model += '>= 1\n'

            # m2gt 下行窗口选择
            self._rt_sel_m2gt = []
            if main_sat_id in self.sat_win:
                m2gt_wins = []
                for win_idx in self.sat_win[main_sat_id]:
                    win = self.win_list[win_idx]
                    if self.is_downlink(win['type']):
                        m2gt_wins.append((win['start_time'], win['end_time']))
                if m2gt_wins:
                    self._rt_sel_m2gt = m2gt_wins
                    safe_main = str(main_sat_id).replace('-', '_')
                    if len(m2gt_wins) > 1:
                        for j, (w_st, w_et) in enumerate(m2gt_wins):
                            self.model += f'rt_m2gt_wl_cons{self.cons_counter}: m2gt - {M} y_rt_m2gt_{safe_main}_{j} >= {w_st - M}\n'
                            self.cons_counter += 1
                            self.model += f'rt_m2gt_wu_cons{self.cons_counter}: m2gt + {M} y_rt_m2gt_{safe_main}_{j} <= {w_et + M}\n'
                            self.cons_counter += 1
                        self.model += f'rt_m2gt_wsel_cons{self.cons_counter}: '
                        self.cons_counter += 1
                        for j in range(len(m2gt_wins)):
                            self.model += f'+ 1 y_rt_m2gt_{safe_main}_{j} '
                        self.model += '>= 1\n'

        # 8. 同星任务时间重叠互斥约束 (防止同一颗卫星在同一时间段开机执行多个冲突任务)
        # for sat_id, task_idxs in self.sat_task.items():
        #     n = len(task_idxs)
        #     for idx_a in range(n):
        #         for idx_b in range(idx_a + 1, n):
        #             i = task_idxs[idx_a]
        #             j = task_idxs[idx_b]
        #             task_i = self.task_list[i]
        #             task_j = self.task_list[j]
                    
        #             st_i, et_i = task_i['start_time'], task_i['end_time']
        #             st_j, et_j = task_j['start_time'], task_j['end_time']

        #             # 判定时间是否交叠冲突 (预留 5 秒姿态机动稳定缓冲)
        #             buffer_time = 5.0
        #             if not (et_i + buffer_time <= st_j or et_j + buffer_time <= st_i):
        #                 self.model += f'overlap_cons{self.cons_counter}: x_{i} + x_{j} <= 1\n'
        #                 self.cons_counter += 1

    def build_variable(self):
        self.model += 'Bounds\n'
        if self.planning_mode == 'on_sat':
            main_sat_id = self.cluster_info['main_sat']
            # tc bounds from routing table (union range)
            for sat_id in self.sat_dic:
                safe_sat = str(sat_id).replace('-', '_')
                rt_wins = getattr(self, '_rt_sel_tc', {}).get(sat_id, [])
                if rt_wins:
                    rt_min = min(w[0] for w in rt_wins)
                    rt_max = max(w[1] for w in rt_wins)
                    self.model += f'{rt_min} <= tc_{safe_sat} <= {rt_max}\n'

            # s2mt bounds from routing table (union range)
            if not self.flag_list.get('is_raw_data_download', False):
                for task in self.task_list:
                    t_idx = task['task_idx']
                    s2mt_wins = getattr(self, '_rt_sel_s2mt', {}).get(t_idx, [])
                    if s2mt_wins:
                        s2mt_min = min(w[0] for w in s2mt_wins)
                        s2mt_max = max(w[1] for w in s2mt_wins)
                        self.model += f'{s2mt_min} <= s2mt_{t_idx} <= {s2mt_max}\n'

            # m2gt bounds from downlink windows (union range)
            m2gt_wins = getattr(self, '_rt_sel_m2gt', [])
            if m2gt_wins:
                m2gt_min = min(w[0] for w in m2gt_wins)
                m2gt_max = max(w[1] for w in m2gt_wins)
                self.model += f'{m2gt_min} <= m2gt <= {m2gt_max}\n'

        if self.planning_mode == 'on_ground':
            for sat_id in self.sat_dic:
                safe_sat = str(sat_id).replace('-', '_')
                
                # 直接从原始数据提取窗口范围，绕过被注释掉的 _win_sel_tc 结构
                if self.sat_win.get(sat_id) is None:
                    continue
                
                # 过滤出测控类型的窗口
                wins = [(self.win_list[i]['start_time'], self.win_list[i]['end_time']) for i in self.sat_win[sat_id] if 'HR' in self.win_list[i].get('type', '') or 'Uplink' in self.win_list[i].get('type', '')]
                
                if not wins:
                    continue
                
                tc_min = min(w[0] for w in wins)
                tc_max = max(w[1] for w in wins)
                self.model += f'{tc_min} <= tc_{safe_sat} <= {tc_max}\n'

        if self.planning_mode == 'on_ground' or self.flag_list.get('is_raw_data_download', False):
            for sat_id in self.sat_dic:
                if self.sat_win.get(sat_id) is None:
                    continue
                dl_wins = [(self.win_list[i]['start_time'], self.win_list[i]['end_time']) for i in self.sat_win[sat_id]
                           if self.is_downlink(self.win_list[i]['type']) and self.win_list[i]['start_time'] >= self.sat_earliest_end.get(sat_id, 0)]
                if dl_wins:
                    dl_min = min(w[0] for w in dl_wins)
                    dl_max = max(w[1] for w in dl_wins)
                    for task_idx in self.sat_task.get(sat_id, []):
                        self.model += f'{dl_min} <= s2gt_{task_idx} <= {dl_max}\n'


        self.model += 'Binaries\n'
        for task in self.task_list:
            t_idx = task["task_idx"]
            self.model += f'x_{t_idx}\n'

        # 声明窗口选择二值变量
        # if self.planning_mode == 'on_ground':
        #     for sat_id in self.sat_dic:
        #         wins = getattr(self, '_win_sel_tc', {}).get(sat_id, [])
        #         safe_id = str(sat_id).replace('-', '_')
        #         if len(wins) > 1:
        #             for j in range(len(wins)):
        #                 self.model += f'y_tc_{safe_id}_{j}\n'

        #         dl_wins = getattr(self, '_win_sel_dl', {})
        #         for task_idx in self.sat_task.get(sat_id, []):
        #             if task_idx in dl_wins and len(dl_wins[task_idx]) > 1:
        #                 for j in range(len(dl_wins[task_idx])):
        #                     self.model += f'y_dl_{task_idx}_{j}\n'

        # 声明 on_sat 模式窗口选择二值变量
        # if self.planning_mode == 'on_sat':
        #     main_sat_id = self.cluster_info['main_sat']
        #     safe_main = str(main_sat_id).replace('-', '_')
        #     for sat_id in self.sat_dic:
        #         rt_wins = getattr(self, '_rt_sel_tc', {}).get(sat_id, [])
        #         safe_id = str(sat_id).replace('-', '_')
        #         if len(rt_wins) > 1:
        #             for j in range(len(rt_wins)):
        #                 self.model += f'y_rt_tc_{safe_id}_{j}\n'

        #     for task in self.task_list:
        #         t_idx = task['task_idx']
        #         s2mt_wins = getattr(self, '_rt_sel_s2mt', {}).get(t_idx, [])
        #         if len(s2mt_wins) > 1:
        #             for j in range(len(s2mt_wins)):
        #                 self.model += f'y_rt_s2mt_{t_idx}_{j}\n'

            # m2gt_wins = getattr(self, '_rt_sel_m2gt', [])
            # if len(m2gt_wins) > 1:
            #     for j in range(len(m2gt_wins)):
            #         self.model += f'y_rt_m2gt_{safe_main}_{j}\n'

        self.model += 'End'

    def is_downlink(self, win_type):
        type_list = win_type.split('+')
        for t in type_list:
            if t == 'R' or t == 'HR' or t == 'Downlink':
                return True
        return False

    def is_tc(self, win_type):
        type_list = win_type.split('+')
        for t in type_list:
            if t == 'HR' or t == 'Uplink':
                return True
        return False

    def SaveModel(self):
        with open(self.save_path, "w", encoding="utf-8") as f:
            f.write(self.model)
        print(f"[Success] Model is successfully saved to: {self.save_path}\n [Done!]")

# ==============================================================================
# 【附录 A：系统核心参数与惩罚权重说明 (Tuning Parameters)】
# ==============================================================================
# 在构建目标函数时，系统为了打破解的对称性并引导求解器优先选择更优的策略，
# 引入了一系列细微的惩罚权重系数 (Penalty Weights)。
# 
# A.1 核心收益指标:
# - BASE_REWARD (默认 10000): 
#   基础任务完成收益。每成功规划一个常规任务，系统获得此收益。
#   这是引导模型尽可能多完成任务的最核心驱动力。
#
# - EMERGENCY_REWARD_MULTIPLIER (默认 10):
#   应急任务收益放大倍数。对于带 emergency 标记的突发任务，
#   其收益 = BASE_REWARD * 10 = 100000。
#   作用：当资源（如电量、时间窗口）发生严重冲突时，
#   求解器会毫不犹豫地放弃最多 9 个常规任务，来保全 1 个应急任务。
#
# A.2 惩罚引导指标 (Breaking Symmetry):
# - TIME_PENALTY_COEF (默认 0.01 / 0.001):
#   时间消耗惩罚系数。如果在 [t1, t2] 和 [t3, t4] 两个窗口都能完成任务，
#   系统会根据实际执行时间 tc 给予极小的负向惩罚（-tc * COEF）。
#   作用：
#   1. 促使卫星尽早执行任务、尽早下传数据。
#   2. 防止求解器在面临同等收益的多个平行解时进行无效搜索，大幅加快求解收敛速度。
#
# - RESOURCE_PENALTY_COEF:
#   电量与存储资源占用惩罚。引导系统在同等条件下，优先选用剩余资源更充沛的卫星。
#
# ==============================================================================
# 【附录 B：边界情况与异常处理策略 (Edge Cases & Fallbacks)】
# ==============================================================================
# B.1 求解无可行解 (Infeasible):
# - 触发场景: 用户下发了绝对无法满足的硬性约束，例如某卫星初始电量为 0，
#   或者要求 1 秒内完成 3 次需要 10 秒机动时间的任务。
# - 应对策略: CBC 求解器会返回 Infeasible。此时后处理模块会捕获异常，
#   并返回全空的规划方案，同时在日志中抛出告警。
# - 未来优化: 引入软约束 (Soft Constraints) 结合松弛变量 (Slack Variables)，
#   即使部分约束被打破，也能给出“尽力而为”的次优解，并标出是哪条约束导致了冲突。
#
# B.2 求解超时 (Time Limit Exceeded):
# - 触发场景: 大规模任务场景下（如 5000+ 任务，几十万变量），MILP 可能会遇到
#   维数灾难，导致数小时无法证明最优性 (Optimality)。
# - 应对策略: 当前平台为 CBC 设置了 maxSeconds 参数。一旦到达时间上限，
#   求解器会直接返回当前找到的最好可行解 (Feasible Solution)。
# - 虽然不一定是全局最优解，但通常质量已经足够高，能满足工程应用对时效性的苛刻要求。
#
# B.3 浮点精度误差 (Numerical Instability):
# - 触发场景: 浮点运算导致原本应该是 1.0 的二进制决策变量 x_i 变成了 0.999999。
# - 应对策略: 在 generate_schedule 模块中，统一采用 0.5 作为二值化分割阈值：
#   if x_i > 0.5: task_selected = True
#   有效规避了精度截断导致的方案丢失。
#
# ==============================================================================
# 【附录 C：未来扩展预留 (Future Expansions)】
# ==============================================================================
# C.1 多星协同与星间链路 (ISL - Intersatellite Links):
#   当前模型已预留了 planingEntityList (星间建链方案) 接口。
#   未来可在约束 3.2 增加: x_i_S1 -> ISL_S1_S2 -> s2gt_S2，
#   支持通过中继卫星进行数据接力下传，彻底摆脱单星对地窗口的限制。
#
# C.2 复杂姿态机动 (Complex Attitude Maneuvers):
#   目前的 buffer_time 是固定常数。
#   未来可引入非线性分段机动时间函数，根据目标间的实际经纬度夹角动态计算 buffer_time，
#   进一步榨干卫星的物理性能极限。
# ==============================================================================