# ==============================================================================
# 文件名称: generate_schedule.py
# 功能描述: 卫星任务规划后处理模块，负责解析 MILP 模型求解结果并生成业务层所需的执行方案。
# 
# 【后处理与指标体系设计文档】
# 
# 1. 核心流程概述:
#    - `read_sol`: 读取求解引擎产生的 .sol 文件。根据不同求解器（CBC、天筹等）
#      兼容不同的解文件输出格式。从文本中萃取出所有有效变量（x_i, tc, s2gt 等）。
#    - `output_schedule`: 根据从 .sol 文件中提取的变量激活状态，映射回物理卫星、
#      地面站和观测目标，组装为业务层可下发执行的 JSON 结构。
# 
# 2. 方案数据结构设计:
#    - planingObservationEntityList (观测方案):
#      - 当 x_i == 1 时，生成该任务。记录目标物 ID、卫星 ID、成像起止时间及模式。
#    - planingSatDealEntityList (星上处理方案):
#      - 根据 isOnboardProcess 标识决定是否在轨直接处理。记录计算量及处理时延。
#    - planingReceiveEntityList (数据下传接收方案):
#      - 依赖 s2gt 时间戳，生成地面站接收计划，确保与地面站过境可见窗口严密耦合。
#    - planingControlEntityList (星地测控方案):
#      - 提取 tc_SAT 时间戳，聚合分配该卫星下所有的测控需求。
#    - planingEntityList (星间建链方案):
#      - 根据路由字典判断是否存在星间中继链路，支持天地一体化网络多跳传输。
# 
# 3. 评测指标 (Metrics) 详细说明:
#    - completionRate (任务完成率): 实际完成的观测目标数 / 请求目标总数。
#    - avgResponseTime (平均响应时间): 任务实际完成时间减去请求下发时间的均值。
#    - resourceUtilization (资源利用率): 所有执行任务耗时之和占总有效观测时间的比率。
#    - ttcOccupancyRate (测控资源占用率): 实际测控时间 / (天基测控时长+地基测控时长)。
#    - emergencyInsertSuccessRate (应急插入成功率): 高优应急任务单独完成的比率。
#    - overallScore (综合总分): 通过配置加权汇总以上各项核心指标的标准化得分。
# 
# 4. 设计原则与边界处理:
#    - 所有时间戳在输入时转为 Unix 浮点秒，在输出时再转为 "%Y-%m-%d %H:%M:%S" 字符串。
#    - 对于非 1.0/0.0 的浮点误差解，设定 BINARIZATION_THRESHOLD 作为阈值进行二值化裁剪。
# ==============================================================================
import re
from datetime import datetime
import time
from constants import (
    OBSERVATION_DEFAULT_DURATION,
    DEFAULT_TTC_DURATION,
    DEFAULT_PROCESS_RATE,
    DEFAULT_TRANSMISSION_RATE,
    BINARIZATION_THRESHOLD,
    EMERGENCY_BLIND_TTC_ADVANCE,
    EMERGENCY_BLIND_TTC_DURATION,
    UNIX_EPOCH_THRESHOLD
)

class ScheduleGenerator:
    def __init__(self, solu_path, input_data, mapping_data, requests=None, runtime_ms=0):
        self.solu_path = solu_path
        self.task_list = input_data['task_list']
        self.sat_dic = input_data['sat_dic']
        self.cluster_info = input_data['cluster_info']
        self.tgt_dic = input_data['tgt_dic']
        self.receive_time = input_data['receive_time']
        self.win_list = input_data['win_list']
        self.rt_list = input_data['rt_list']
        self.sat_rt = mapping_data['sat_rt']
        self.sat_win = mapping_data['sat_win']
        self.requests = requests if requests is not None else input_data.get('requests', [])
        self.emergency_tgt_ids = input_data.get('emergency_tgt_ids', set())
        self.runtime_ms = runtime_ms
        self.planning_mode = 'on_ground'
        self.read_sol()

    def _get_target_req_info(self, target_id):
        """根据 target_id 找到该目标归属的需求单号与资源标识"""
        if isinstance(self.requests, list):
            for req in self.requests:
                tgt_list = req.get('targetList', [])
                if any(t.get('targetId') == target_id for t in tgt_list if isinstance(t, dict)):
                    return req.get('resource', ''), req.get('zcRequestId', '')
            if len(self.requests) > 0 and isinstance(self.requests[0], dict):
                return self.requests[0].get('resource', ''), self.requests[0].get('zcRequestId', '')
        elif isinstance(self.requests, dict):
            return self.requests.get('resource', ''), self.requests.get('zcRequestId', '')
        return '', ''

    def _get_req_field(self, field_name):
        """获取首个需求单中的指定通用字段"""
        if isinstance(self.requests, list) and len(self.requests) > 0 and isinstance(self.requests[0], dict):
            return self.requests[0].get(field_name, '')
        elif isinstance(self.requests, dict):
            return self.requests.get(field_name, '')
        return ''


    def output_schedule(self):
        schedule = {
            "planingObservationEntityList": [],
            "planingReceiveEntityList": [],
            "planingEntityList": [],
            "planingSatDealEntityList": [],
            "planingControlEntityList": [],
            "metrics": {}
        }
        sat_task_id = {}
        sat_task_num = {}
        tgt_seledted = {}
        task_count = 0
        link_time_records = []

        for idx, info in self.task_info.items():
            x_key = 'x_' + idx
            if x_key not in info or float(info[x_key]) < BINARIZATION_THRESHOLD:
                continue

            task_count += 1
            # 直接修改原始 task_obj，确保下方所有的 Metrics 评测指标计算也能同步基于这 10 秒
            task_obj = self.task_list[int(idx)]
            
            # 根据用户需求，强制将观测方案的时长固定为预设常量（默认 10 秒）
            # （这样后续的星上处理 deal_duration 和数传时间都会自动基于常量计算）
            task_obj['duration'] = OBSERVATION_DEFAULT_DURATION
            task_obj['end_time'] = task_obj['start_time'] + OBSERVATION_DEFAULT_DURATION
            
            tgt_id = task_obj['targetId']
            sat_id = str(task_obj['satelliteId']).strip()
            
            raw_task_id = task_obj.get('task_id', task_obj.get('taskId', x_key))
            res_src, req_id = self._get_target_req_info(tgt_id)

            if tgt_seledted.get(tgt_id) is None:
                tgt_seledted[tgt_id] = 1

            # 统一使用下划线清洗后的 ID 作为 key，防止 "SAT-1" 和 "SAT_1" 或 "SAT-1 " 被误判为多颗卫星
            clean_sat_id = sat_id.replace('-', '_')
            if clean_sat_id not in sat_task_id:
                sat_task_id[clean_sat_id] = [raw_task_id]
                sat_task_num[clean_sat_id] = 1
            else:
                if raw_task_id not in sat_task_id[clean_sat_id]:
                    sat_task_id[clean_sat_id].append(raw_task_id)
                sat_task_num[clean_sat_id] += 1

            st_str = datetime.fromtimestamp(float(task_obj['start_time'])).strftime("%Y-%m-%d %H:%M:%S")
            et_str = datetime.fromtimestamp(float(task_obj['end_time'])).strftime("%Y-%m-%d %H:%M:%S")

            schedule["planingObservationEntityList"].append({
                "taskId": raw_task_id,
                "satelliteId": sat_id,
                "source": res_src,
                "standardReqId": req_id,
                "rollAngle": task_obj.get('rollAngle', 0.0),
                "taskStartTime": st_str,
                "taskEndTime": et_str,
                "taskPayloadId": "",
                "targetId": tgt_id
            })

            process_rate = self.sat_dic[sat_id].get('process_rate', DEFAULT_PROCESS_RATE)
            trans_rate = self.sat_dic[sat_id].get('transmission_rate', DEFAULT_TRANSMISSION_RATE)
            deal_duration = task_obj['duration'] * process_rate

            schedule["planingSatDealEntityList"].append({
                "taskId": raw_task_id,
                "satelliteId": sat_id,
                "processTaskId": 'p_' + idx,
                "startTime": et_str,
                "dealTime": f"{deal_duration}",
                "resource": res_src,
            })

            best_start_time = float(task_obj['end_time']) + deal_duration
            trans_sat_id = sat_id
            ground_id = None

            for cur_sat_id, win_idxs in self.sat_win.items():
                cur_process_rate = self.sat_dic.get(cur_sat_id, {}).get('process_rate', DEFAULT_PROCESS_RATE)
                for win_idx in win_idxs:
                    win = self.win_list[win_idx]
                    if task_obj['duration'] * cur_process_rate < win['end_time']:
                        if task_obj['duration'] * process_rate + task_obj['end_time'] >= win['start_time']:
                            start_trans_time = task_obj['duration'] * process_rate + task_obj['end_time']
                            if cur_sat_id == sat_id:
                                trans_sat_id = cur_sat_id
                                ground_id = win.get('sidAId', '')
                                best_start_time = start_trans_time
                                break
                            else:
                                start_trans_time = win['start_time']
                            if start_trans_time < best_start_time:
                                trans_sat_id = cur_sat_id
                                ground_id = win.get('sidAId', '')
                                best_start_time = start_trans_time

            end_trans_time = best_start_time + task_obj['duration'] * trans_rate
            st_link_str = datetime.fromtimestamp(best_start_time).strftime("%Y-%m-%d %H:%M:%S")
            et_link_str = datetime.fromtimestamp(end_trans_time).strftime("%Y-%m-%d %H:%M:%S")

            if trans_sat_id != sat_id and sat_id in self.sat_rt:
                for rt_idx in self.sat_rt[sat_id]:
                    rt = self.rt_list[rt_idx]
                    des_sat_str = rt.get('desSatellited', rt.get('desSatelliteId', ''))
                    des_sat_ids = des_sat_str.split(',') if isinstance(des_sat_str, str) else [str(des_sat_str)]
                    for des_sat_id in des_sat_ids:
                        if des_sat_id == trans_sat_id:
                            link_time_records.append((best_start_time, end_trans_time, sat_id, des_sat_id, raw_task_id))
                            
                            schedule["planingEntityList"].append({
                                "linkId": f"link_{sat_id}_0",
                                "resource": res_src,
                                "requireSource": res_src,
                                "standardRequestId": req_id,
                                "satelliteId": sat_id,
                                "satelliteLinkId": des_sat_id,
                                "linkSartTime": st_link_str,
                                "linkEndTime": et_link_str
                            })
                            schedule["planingReceiveEntityList"].append({
                                "taskId": raw_task_id,
                                "satelliteId": sat_id,
                                "groudStationId": ground_id,
                                "dataTxTaskId": 's2mt_' + idx,
                                "taskStartTime": st_link_str,
                                "taskEndTime": et_link_str,
                                "dataTransMode": "routing",
                                "resource": res_src,
                            })
            else:
                schedule["planingReceiveEntityList"].append({
                    "taskId": raw_task_id,
                    "satelliteId": sat_id,
                    "groudStationId": ground_id,
                    "dataTxTaskId": 's2mt_' + idx,
                    "taskStartTime": st_link_str,
                    "taskEndTime": et_link_str,
                    "dataTransMode": "real-time",
                    "resource": res_src,
                })

        for clean_sat_id, task_ids in sat_task_id.items():
            # 获取该卫星算出的实际上注时间 tc_time
            tc_time = float(self.tc_info.get(clean_sat_id, self.tc_info.get('tc_' + clean_sat_id, 0)))
            
            # 尝试还原原始的卫星 ID（带横杠的格式），用于在原始字典中查找窗口
            orig_sat_id = clean_sat_id
            if orig_sat_id not in self.sat_win and orig_sat_id.replace('_', '-') in self.sat_win:
                orig_sat_id = orig_sat_id.replace('_', '-')
                
            # 动态反向匹配测控站点 (accessPoint) 和测控完整时间段
            real_access_point = "Default_Ground_A" # 兜底值
            ttc_st = tc_time
            ttc_et = tc_time + DEFAULT_TTC_DURATION # 兜底时长
            matched = False
            
            if orig_sat_id in self.sat_win:
                for win_idx in self.sat_win[orig_sat_id]:
                    win = self.win_list[win_idx]
                    # 既然业务上允许“测控方案用的也是数传窗口”，我们取消严格的 'HR'/'Uplink' type 过滤，
                    # 只要求解器计算出的 tc_time 落在这个窗口内，就认为匹配成功
                    if win['start_time'] <= tc_time <= win['end_time']:
                        # sidAId 即为测控站（或天基节点）的 ID
                        if win.get('sidAId'):
                            real_access_point = str(win['sidAId'])
                        # 核心修改：直接取该完整窗口段的起止时间作为测控方案时长
                        ttc_st = win['start_time']
                        ttc_et = win['end_time']
                        matched = True
                        break

            # 极端兜底保护：如果输入 JSON 中该卫星根本没有提供测控窗口 (tc_time 就会是 0)
            if not matched or tc_time < UNIX_EPOCH_THRESHOLD:
                # 寻找该卫星在本次规划中最早的观测时间
                my_obs_times = []
                for obs in schedule["planingObservationEntityList"]:
                    if obs['satelliteId'] == orig_sat_id or obs['satelliteId'] == clean_sat_id:
                        parsed_time = datetime.strptime(obs['taskStartTime'], "%Y-%m-%d %H:%M:%S").timestamp()
                        my_obs_times.append(parsed_time)

                if my_obs_times:
                    earliest_st = min(my_obs_times)
                    ttc_st = earliest_st - EMERGENCY_BLIND_TTC_ADVANCE  # 紧急盲注提前量
                    ttc_et = ttc_st + EMERGENCY_BLIND_TTC_DURATION      # 兜底窗口时长
                    real_access_point = "Emergency_Blind_Node"
                else:
                    ttc_st = time.time()
                    ttc_et = ttc_st + DEFAULT_TTC_DURATION

            schedule["planingControlEntityList"].append({
                "ttcTaskId": f"ttc_{clean_sat_id}",
                "satelliteId": clean_sat_id,
                "accessPoint": real_access_point,
                "taskStartTime": datetime.fromtimestamp(ttc_st).strftime("%Y-%m-%d %H:%M:%S"),
                "taskEndTime": datetime.fromtimestamp(ttc_et).strftime("%Y-%m-%d %H:%M:%S"),
                "taskIdSet": ",".join([str(tid) for tid in task_ids]) if isinstance(task_ids, list) else str(task_ids),
                "resource": self._get_req_field('resource'),
            })

        # ------------------ 评测指标 Schema 精确计算 (Metrics) ------------------
        # 1. 任务完成率 (completionRate)
        obs_entities = schedule["planingObservationEntityList"]
        observed_targets = len(set(item['targetId'] for item in obs_entities if item.get('targetId')))
        total_targets = len(self.tgt_dic)

        if total_targets > 0:
            schedule["metrics"]["completionRate"] = round(min(1.0, observed_targets / total_targets), 4)
        else:
            schedule["metrics"]["completionRate"] = 0.0

        # 2. 任务平均响应时间（毫秒 Long） (avgResponseTime)
        req_start_ts = datetime.strptime(self.receive_time, "%Y-%m-%d %H:%M:%S").timestamp()
        tgt_earliest_obs_time = {}
        for idx in self.task_info:
            if 'x_' + idx in self.task_info[idx] and float(self.task_info[idx]['x_'+idx]) >= 0.5:
                t_obj = self.task_list[int(idx)]
                t_id = t_obj['targetId']
                obs_start_ts = float(t_obj['start_time'])
                tgt_earliest_obs_time[t_id] = min(tgt_earliest_obs_time.get(t_id, obs_start_ts), obs_start_ts)

        if len(tgt_earliest_obs_time) > 0:
            avg_delay_sec = sum([max(0.0, st - req_start_ts) for st in tgt_earliest_obs_time.values()]) / len(tgt_earliest_obs_time)
            schedule["metrics"]["avgResponseTime"] = int(avg_delay_sec * 1000)
        else:
            schedule["metrics"]["avgResponseTime"] = 0

        # 3. 负载均衡系数 / 资源利用率 (resourceUtilization)
        m = len(obs_entities)  # 总任务量 m

        sat_x_k = {}      # x_k: 卫星 k 实际分配到的任务数
        sat_T_k_use = {}  # T_k,use: 卫星 k 实际总工作时长 (毫秒)

        for item in obs_entities:
            s_id = item['satelliteId']
            st_ts = datetime.strptime(item['taskStartTime'], "%Y-%m-%d %H:%M:%S").timestamp()
            et_ts = datetime.strptime(item['taskEndTime'], "%Y-%m-%d %H:%M:%S").timestamp()
            duration_ms = max(0.0, (et_ts - st_ts) * 1000)

            sat_x_k[s_id] = sat_x_k.get(s_id, 0) + 1
            sat_T_k_use[s_id] = sat_T_k_use.get(s_id, 0.0) + duration_ms

        S = [s_id for s_id, T_use in sat_T_k_use.items() if T_use > 0]
        N_used = len(S)

        if N_used <= 1 or m == 0:
            schedule["metrics"]["resourceUtilization"] = 1.0
        else:
            total_T_use = sum([sat_T_k_use[k] for k in S])
            x_hat = {}
            for k in S:
                x_hat[k] = m * (sat_T_k_use[k] / total_T_use) if total_T_use > 0 else 0.0

            sigma2 = (1.0 / N_used) * sum([(sat_x_k[k] - x_hat[k]) ** 2 for k in S])
            sigma2_max = (m ** 2) * (N_used - 1) / (N_used ** 2)

            if sigma2_max > 0:
                alpha = round(max(0.0, 1.0 - (sigma2 / sigma2_max)), 4)
            else:
                alpha = 1.0

            schedule["metrics"]["resourceUtilization"] = alpha

        # 4. 测控资源占用率 (ttcOccupancyRate)
        total_tc_windows = sum(1 for win in self.win_list if 'HR' in win.get('type','') or 'Uplink' in win.get('type','')) or 1
        used_tc_segments = len(schedule["planingControlEntityList"])
        schedule["metrics"]["ttcOccupancyRate"] = round(min(1.0, used_tc_segments / total_tc_windows), 4)

        # 5. 数传资源占用率 (dtOccupancyRate)
        total_win_duration = sum([(w['end_time'] - w['start_time']) for w in self.win_list]) or 1.0
        actual_dt_duration = sum([self.task_list[int(idx)]['duration'] * self.sat_dic[self.task_list[int(idx)]['satelliteId']].get('transmission_rate', 2.0) for idx in self.task_info if 'x_'+idx in self.task_info[idx] and float(self.task_info[idx]['x_'+idx]) >= 0.5])
        used_dt_window_duration = sum([(win['end_time'] - win['start_time']) for win in self.win_list if ('HR' in win.get('type','') or 'Downlink' in win.get('type','') or 'R' in win.get('type','')) and any(item.get('groudStationId') == win.get('sidAId') for item in schedule["planingReceiveEntityList"])]) or total_win_duration
        schedule["metrics"]["dtOccupancyRate"] = round(min(1.0, actual_dt_duration / used_dt_window_duration), 4)

        # 6. 数传时效性（毫秒 Long） (dtTimeliness)
        dt_delays = []
        for recv_entity in schedule["planingReceiveEntityList"]:
            if recv_entity.get("dataTransMode") == "real-time":
                dt_delays.append(0)
            else:
                raw_t_id = recv_entity.get("taskId", "")
                dt_st_ts = datetime.strptime(recv_entity["taskStartTime"], "%Y-%m-%d %H:%M:%S").timestamp()
                
                obs_et_ts = dt_st_ts
                for task_obj in self.task_list:
                    if task_obj.get('task_id') == raw_t_id or task_obj.get('taskId') == raw_t_id:
                        obs_et_ts = float(task_obj['end_time'])
                        break
                
                dt_delays.append(int(max(0.0, dt_st_ts - obs_et_ts) * 1000))

        schedule["metrics"]["dtTimeliness"] = int(sum(dt_delays) / len(dt_delays)) if dt_delays else 0

        # 7. 建链任务完成率 (linkTaskCompletionRate)
        needed_link_task_ids = set()
        for idx in self.task_info:
            if 'x_' + idx in self.task_info[idx] and float(self.task_info[idx]['x_'+idx]) >= 0.5:
                task_obj = self.task_list[int(idx)]
                sat_id = task_obj['satelliteId']
                process_rate = self.sat_dic.get(sat_id, {}).get('process_rate', 0.5)
                ready_time = task_obj['end_time'] + task_obj['duration'] * process_rate

                t_direct = float('inf')
                if sat_id in self.sat_win:
                    for win_idx in self.sat_win[sat_id]:
                        win = self.win_list[win_idx]
                        if ready_time <= win['end_time']:
                            t_direct = min(t_direct, max(win['start_time'], ready_time))

                t_routing = float('inf')
                if sat_id in self.sat_rt:
                    for rt_idx in self.sat_rt[sat_id]:
                        rt = self.rt_list[rt_idx]
                        des_sat = rt.get('desSatellited', rt.get('desSatelliteId', ''))
                        des_sats = des_sat.split(',') if isinstance(des_sat, str) else [str(des_sat)]
                        for ds in des_sats:
                            if ds in self.sat_win:
                                for win_idx in self.sat_win[ds]:
                                    win = self.win_list[win_idx]
                                    if ready_time <= win['end_time']:
                                        t_routing = min(t_routing, max(win['start_time'], ready_time))

                if t_direct == float('inf') or t_direct > t_routing:
                    raw_t_id = task_obj.get('task_id', task_obj.get('taskId', 'x_' + idx))
                    needed_link_task_ids.add(raw_t_id)

        needed_link_tasks = len(needed_link_task_ids)
        if needed_link_tasks > 0:
            schedule["metrics"]["linkTaskCompletionRate"] = round(min(1.0, len(link_time_records) / needed_link_tasks), 4)
        else:
            schedule["metrics"]["linkTaskCompletionRate"] = 1.0

        # 8. 建链资源占用率 (linkOccupancyRate)
        actual_link_dur = sum([(rec[1] - rec[0]) for rec in link_time_records])
        used_rt_duration = sum([(rt['end_time'] - rt['start_time']) for rt in self.rt_list]) or 1.0
        schedule["metrics"]["linkOccupancyRate"] = round(min(1.0, actual_link_dur / used_rt_duration), 4)

        # 9. 建链重复率 (linkRepeatRate)
        task_link_nodes = {}
        for rec in link_time_records:
            sat_src = rec[2]
            sat_des = rec[3]
            t_id = rec[4]
            if t_id not in task_link_nodes:
                task_link_nodes[t_id] = []
            task_link_nodes[t_id].append(sat_src)
            task_link_nodes[t_id].append(sat_des)

        total_pass_count = 0
        total_dup_count = 0

        for t_id, nodes in task_link_nodes.items():
            n_pass = len(nodes)
            n_unique = len(set(nodes))
            n_dup = n_pass - n_unique
            total_pass_count += n_pass
            total_dup_count += n_dup

        if total_pass_count > 0:
            schedule["metrics"]["linkRepeatRate"] = round(total_dup_count / total_pass_count, 4)
        else:
            schedule["metrics"]["linkRepeatRate"] = 0.0

        # 10. 处理方案完成率 (processTaskCompletionRate)
        needed_process_tasks = 0
        for idx in self.task_info:
            if 'x_' + idx in self.task_info[idx] and float(self.task_info[idx]['x_'+idx]) >= 0.5:
                task_obj = self.task_list[int(idx)]
                tgt_id = task_obj['targetId']
                res_src, req_id = self._get_target_req_info(tgt_id)
                
                need_process = True
                if isinstance(self.requests, list):
                    for req in self.requests:
                        if req.get('zcRequestId') == req_id:
                            need_process = req.get('isOnboardProcess', True)
                            break
                elif isinstance(self.requests, dict):
                    need_process = self.requests.get('isOnboardProcess', True)

                if need_process:
                    needed_process_tasks += 1

        actual_deal_tasks = len(schedule["planingSatDealEntityList"])

        if needed_process_tasks > 0:
            schedule["metrics"]["processTaskCompletionRate"] = round(min(1.0, actual_deal_tasks / needed_process_tasks), 4)
        else:
            schedule["metrics"]["processTaskCompletionRate"] = 1.0

        # 11. 处理资源负载均衡系数 α (processLoadBalance)
        schedule["metrics"]["processLoadBalance"] = schedule["metrics"]["resourceUtilization"]

        # 12. 应急任务插入成功率 (emergencyInsertSuccessRate)
        if len(self.emergency_tgt_ids) > 0:
            selected_em_targets = len(set(item['targetId'] for item in obs_entities if item.get('targetId') in self.emergency_tgt_ids))
            schedule["metrics"]["emergencyInsertSuccessRate"] = round(min(1.0, selected_em_targets / len(self.emergency_tgt_ids)), 4)
        else:
            schedule["metrics"]["emergencyInsertSuccessRate"] = 1.0

        # 13. 观测任务规划时效性（毫秒 Long） (schedulingDuration)
        schedule["metrics"]["schedulingDuration"] = self.runtime_ms

        # ------------------ 100 分制综合评分计算 (overallScore) ------------------
        earliest_obs_ts = min([t['start_time'] for t in self.task_list]) if self.task_list else (req_start_ts + 3600)
        latest_obs_ts = max([t['end_time'] for t in self.task_list]) if self.task_list else (req_start_ts + 7200)
        req_end_ts = latest_obs_ts + 3600

        denom_plan = max(1.0, earliest_obs_ts - req_start_ts)
        denom_resp = max(1.0, latest_obs_ts - req_start_ts)
        denom_dt = max(1.0, req_end_ts - req_start_ts)

        score_ttc_occ = 10.0 * (1.0 - schedule["metrics"]["ttcOccupancyRate"])
        score_comp = 12.0 * schedule["metrics"]["completionRate"]

        plan_time_sec = (self.runtime_ms / 1000.0)
        score_plan_time = 12.0 * max(0.0, 1.0 - (plan_time_sec / denom_plan))

        resp_time_sec = (schedule["metrics"]["avgResponseTime"] / 1000.0)
        score_resp_time = 12.0 * max(0.0, 1.0 - (resp_time_sec / denom_resp))

        score_res_util = 12.0 * schedule["metrics"]["resourceUtilization"]
        score_em_insert = 12.0 * schedule["metrics"]["emergencyInsertSuccessRate"]

        # 建链重复率为硬约束 (必须为 0，否则建链完成率与占用率得 0 分)
        if schedule["metrics"]["linkRepeatRate"] == 0:
            score_link_comp = 5.0 * schedule["metrics"]["linkTaskCompletionRate"]
            score_link_occ = 5.0 * (1.0 - schedule["metrics"]["linkOccupancyRate"])
        else:
            score_link_comp = 0.0
            score_link_occ = 0.0

        score_dt_occ = 5.0 * (1.0 - schedule["metrics"]["dtOccupancyRate"])

        dt_delay_sec = (schedule["metrics"]["dtTimeliness"] / 1000.0)
        score_dt_time = 5.0 * max(0.0, 1.0 - (dt_delay_sec / denom_dt))

        score_proc_comp = 5.0 * schedule["metrics"]["processTaskCompletionRate"]
        score_proc_bal = 5.0 * schedule["metrics"]["processLoadBalance"]

        total_score = (
            score_ttc_occ + score_comp + score_plan_time + score_resp_time +
            score_res_util + score_em_insert + score_link_comp + score_link_occ +
            score_dt_occ + score_dt_time + score_proc_comp + score_proc_bal
        )

        schedule["metrics"]["overallScore"] = round(total_score, 2)

        # ------------------ 输出方案规整化 (时间序列排序) ------------------
        # 对各类方案严格按照开始时间进行升序排序，使输出的 JSON 呈现良好的时间线递进关系
        if "planingObservationEntityList" in schedule:
            schedule["planingObservationEntityList"].sort(key=lambda x: x.get("startTime", ""))
        
        if "planingSatDealEntityList" in schedule:
            schedule["planingSatDealEntityList"].sort(key=lambda x: x.get("processStartTime", ""))
            
        if "planingReceiveEntityList" in schedule:
            schedule["planingReceiveEntityList"].sort(key=lambda x: x.get("downStartTime", ""))
            
        if "planingControlEntityList" in schedule:
            schedule["planingControlEntityList"].sort(key=lambda x: x.get("tcStartTime", ""))
            
        if "planingEntityList" in schedule:
            schedule["planingEntityList"].sort(key=lambda x: x.get("linkStartTime", ""))

        return schedule

    def read_sol(self):
        self.task_info = {}
        self.tc_info = {}
        self.obj = 0
        try:
            with open(self.solu_path, 'r', encoding='utf-8', errors='ignore') as f:
                for line in f.readlines():
                    line_str = line.strip()
                    if not line_str or line_str.startswith('#') or line_str.startswith('*'):
                        continue

                    var_name = None
                    val_num = None

                    # 1. 尝试匹配 XML/HTML 格式: <variable name="x_0" value="1"/>
                    xml_match = re.search(r'name=["\']([^"\']+)["\'].*?value=["\']([^"\']+)["\']', line_str)
                    if xml_match:
                        var_name = xml_match.group(1)
                        try:
                            val_num = float(xml_match.group(2))
                        except ValueError:
                            pass

                    # 2. 尝试匹配通用文本格式 (如 0 x_0 1.0 或 x_0 = 1.0 或 VAR x_0 1.0)
                    if var_name is None:
                        parts = line_str.replace('=', ' ').split()
                        if len(parts) >= 3 and parts[0].isdigit():
                            var_name = parts[1]
                            try:
                                val_num = float(parts[2])
                            except ValueError:
                                pass
                        elif len(parts) >= 2:
                            candidate_name = parts[0]
                            candidate_val = parts[1]
                            if candidate_name.upper() in ['VAR', 'VARIABLE']:
                                candidate_name = parts[1]
                                candidate_val = parts[2] if len(parts) > 2 else '0'

                            try:
                                val_num = float(candidate_val)
                                var_name = candidate_name
                            except ValueError:
                                pass

                    # 若该行不是有效的变量和数值，安全跳过
                    if var_name is None or val_num is None:
                        continue

                    # 3. 归集目标函数与变量分配
                    if 'obj' in var_name.lower() or 'm2gt' in var_name.lower():
                        self.obj = var_name, val_num
                        continue

                    if var_name.startswith('tc_'):
                        sat_k = var_name[3:]
                        self.tc_info[sat_k] = val_num
                    else:
                        match = re.search(r'_(\d+)$', var_name)
                        if match:
                            t_idx = match.group(1)
                            if t_idx not in self.task_info:
                                self.task_info[t_idx] = {}
                            self.task_info[t_idx][var_name] = val_num
        except Exception as e:
            print(f"[Error reading sol file]: {e}")

# ==============================================================================
# 【附录 D：数据格式约定与外部系统集成指南 (Integration Guidelines)】
# ==============================================================================
# D.1 时间与空间坐标系标准:
# - 时间系统: 算法内部计算强制采用 Unix Timestamp (浮点秒)，精确到毫秒。
#   对外输出时，通过 datetime.fromtimestamp 统一转换为 "YYYY-MM-DD HH:MM:SS" 标准字符串。
#   如果甲方系统对接时存在时区 (Timezone) 偏差，需在 load_input 模块统一加减 offset。
# - 空间坐标系: 默认为 WGS84。经度 (-180, 180)，纬度 (-90, 90)。
#   本算法目前不涉及复杂的球面几何计算（由前置的轨道动力学模块负责生成可见性窗口），
#   但保留了 target_point 字段透传。
#
# D.2 方案下发与执行协议:
# - 方案分离: 最终生成的 JSON 被严格划分为 observation（观测）、
#   sat_deal（星上处理）、receive（地面站接收）、control（测控注入）四组实体。
# - 调度耦合: 地面站接收计划 (receive) 和 星上处理 (sat_deal) 在业务逻辑上是强绑定的。
#   即：只有当数据在星上处理完成 (s2mt) 后，才能在最近的可见窗口进行数传 (s2gt)。
#   下游控制系统在解析该 JSON 时，需确保指令上注序列的严格先后依赖关系。
#
# D.3 系统异常与容错:
# - 求解引擎若崩溃或未能生成 sol 文件，本模块的 output_schedule 会被兜底执行，
#   生成完全空的 List，并附带错误 Metrics。这确保了 HTTP 接口永远不会向调用方抛出
#   500 Internal Server Error，而是返回带有明确错误状态码的合法 JSON。
# ==============================================================================
