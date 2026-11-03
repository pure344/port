# ==============================================================================
# 文件名称: generate_schedule.py
# 功能描述: 卫星任务规划后处理与 DDS 标准报文生成引擎
#          负责解析 CBC 求解器产生的 .sol 结果，提取决策变量激活状态，
#          并严格按照星间 DDS 数据架构规范输出：
#          1. 侦察任务分配数据 (ZC_TASK_ASSIGN_TOPIC / zc_task_assign.idl)
#          2. 预警任务分配数据 (YJ_TASK_ASSIGN_TOPIC / yj_task_assign.idl)
#          3. 单星任务指令数据 (SINSATE_COMMAND_TOPIC / sinsate_command.idl)
#          4. 规划效能评测指标 (metrics)
# ==============================================================================

import re
import time
from typing import Dict, Any, List
from constants import (
    BINARIZATION_THRESHOLD,
    OBSERVATION_DEFAULT_DURATION,
    format_ts_to_iso,
    euler_to_quaternion,
    make_dds_wrapper,
    TOPIC_ZC_TASK_ASSIGN,
    TOPIC_YJ_TASK_ASSIGN,
    TOPIC_SINSATE_COMMAND
)

class ScheduleGenerator:
    def __init__(self, solu_path: str, input_data: Dict[str, Any], mapping_data: Dict[str, Any], raw_requests=None, runtime_ms: int = 0):
        self.solu_path = solu_path
        self.input_data = input_data
        self.mission_mode = input_data.get('mission_mode', 'zc')
        self.task_list = input_data['task_list']
        self.sat_dic = input_data['sat_dic']
        self.tgt_dic = input_data['tgt_dic']
        self.demand_info = input_data.get('demand_info', {})
        self.receive_time = input_data.get('receive_time', time.time())
        self.win_list = input_data.get('win_list', [])
        self.rt_list = input_data.get('rt_list', [])
        self.sat_task = mapping_data['sat_task']
        self.sat_win = mapping_data['sat_win']
        self.runtime_ms = runtime_ms

        self.task_info: Dict[str, Dict[str, float]] = {}
        self.tc_info: Dict[str, float] = {}
        self.read_sol()

    def output_schedule(self) -> Dict[str, Any]:
        """
        生成严格符合 DDS 标准主题报文格式的规划结果
        """
        assign_topic = TOPIC_ZC_TASK_ASSIGN if self.mission_mode == 'zc' else TOPIC_YJ_TASK_ASSIGN
        domain_key = "RECON" if self.mission_mode == 'zc' else "WARNING"

        schedule_res = {
            "missionMode": self.mission_mode,
            "taskAssignTopic": assign_topic,
            "commandTopic": TOPIC_SINSATE_COMMAND,
            "taskAssignList": [],
            "commandList": [],
            "metrics": {}
        }

        # 统计被激活执行的任务
        active_tasks = []
        observed_tgt_ids = set()

        for idx_str, var_dict in self.task_info.items():
            x_key = f"x_{idx_str}"
            if x_key in var_dict and float(var_dict[x_key]) >= BINARIZATION_THRESHOLD:
                t_idx = int(idx_str)
                if t_idx < len(self.task_list):
                    t_obj = self.task_list[t_idx]
                    active_tasks.append(t_obj)
                    observed_tgt_ids.add(t_obj['tgt_id'])

        assign_time_iso = format_ts_to_iso(time.time())

        # ----------------------------------------------------------------------
        # 1. 构建任务分配数据 (zc_task_assign 或 yj_task_assign)
        # ----------------------------------------------------------------------
        # 按卫星 SCID 归拢执行方案
        sat_assigned_tasks: Dict[int, List[Dict[str, Any]]] = {}
        for t in active_tasks:
            scid = int(t.get('scid', 1))
            sat_assigned_tasks.setdefault(scid, []).append(t)

        mission_counter = 1

        if self.mission_mode == 'zc':
            # --- 侦察任务分配报文 (ZC_TASK_ASSIGN_TOPIC) ---
            for scid, t_list in sat_assigned_tasks.items():
                t_st = min(t['start_time'] for t in t_list)
                t_et = max(t['end_time'] for t in t_list)

                assigned_pts = []
                assigned_pt_types = []
                assigned_pt_prios = []
                assigned_pt_loads = []

                assigned_areas_pot1 = []
                assigned_areas_pot2 = []
                assigned_areas_pot3 = []
                assigned_areas_pot4 = []
                assigned_area_types = []
                assigned_area_prios = []
                assigned_area_loads = []

                for t in t_list:
                    tgt_id = t['tgt_id']
                    tgt_meta = self.tgt_dic.get(tgt_id, {})
                    if tgt_meta.get('type') == 'point':
                        pt_pos = tgt_meta.get('pos', {
                            "targetId": tgt_id,
                            "lon": 0.0,
                            "lat": 0.0,
                            "alt": 0.0
                        })
                        assigned_pts.append(pt_pos)
                        assigned_pt_types.append(tgt_meta.get('category', 'Unknown'))
                        assigned_pt_prios.append(int(tgt_meta.get('priority', 1)))
                        assigned_pt_loads.append(tgt_meta.get('payload', 'Panchromatic'))
                    elif tgt_meta.get('type') == 'area':
                        verts = tgt_meta.get('vertices', [{}, {}, {}, {}])
                        assigned_areas_pot1.append(verts[0] if len(verts) > 0 else {})
                        assigned_areas_pot2.append(verts[1] if len(verts) > 1 else {})
                        assigned_areas_pot3.append(verts[2] if len(verts) > 2 else {})
                        assigned_areas_pot4.append(verts[3] if len(verts) > 3 else {})
                        assigned_area_types.append(tgt_meta.get('category', 'Unknown'))
                        assigned_area_prios.append(int(tgt_meta.get('priority', 1)))
                        assigned_area_loads.append(tgt_meta.get('payload', 'Panchromatic'))

                zc_assign_data = {
                    "execNode": scid,
                    "missionID": mission_counter,
                    "taskAssignTime": assign_time_iso,
                    "taskStartTime": format_ts_to_iso(t_st),
                    "taskEndTime": format_ts_to_iso(t_et),
                    "pointTargetCnt": len(assigned_pts),
                    "pointTargetPos": assigned_pts,
                    "pointTargetType": assigned_pt_types,
                    "pointTargetPriority": assigned_pt_prios,
                    "pointTargetPayload": assigned_pt_loads,
                    "areaTargetCnt": len(assigned_areas_pot1),
                    "areaPot1Pos": assigned_areas_pot1,
                    "areaPot2Pos": assigned_areas_pot2,
                    "areaPot3Pos": assigned_areas_pot3,
                    "areaPot4Pos": assigned_areas_pot4,
                    "areaTargetType": assigned_area_types,
                    "areaTargetPriority": assigned_area_prios,
                    "areaTargetPayload": assigned_area_loads
                }
                mission_counter += 1

                # 封装为 DDS 标准报文
                dds_msg = make_dds_wrapper(
                    TOPIC_ZC_TASK_ASSIGN,
                    domain_key,
                    f"侦察任务分配数据-SCID{scid}",
                    zc_assign_data
                )
                schedule_res["taskAssignList"].append(dds_msg)

        else:
            # --- 预警任务分配报文 (YJ_TASK_ASSIGN_TOPIC) ---
            raw_req = self.demand_info.get('raw_req', {})
            task_type = self.demand_info.get('taskType', 'Staring')

            for scid, t_list in sat_assigned_tasks.items():
                t_st = min(t['start_time'] for t in t_list)
                t_et = max(t['end_time'] for t in t_list)

                # 轨迹预测与位置
                pts = raw_req.get('pointPosList', [])
                vels = raw_req.get('pointVelList', [])
                times = raw_req.get('pointTime', [])

                real_time = times[0] if times else assign_time_iso
                real_pos = pts[0] if pts else {"lon": float(raw_req.get('viewLon', 0.0)), "lat": float(raw_req.get('viewLat', 0.0)), "alt": float(raw_req.get('viewAlt', 0.0))}
                real_vel = vels[0] if vels else {"eastVel": 0.0, "northVel": 0.0, "upVel": 0.0}

                pred_times = times[1:] if len(times) > 1 else []
                pred_pts = pts[1:] if len(pts) > 1 else []
                pred_vels = vels[1:] if len(vels) > 1 else []

                yj_assign_data = {
                    "execNode": scid,
                    "missionID": mission_counter,
                    "taskAssignTime": assign_time_iso,
                    "taskStartTime": format_ts_to_iso(t_st),
                    "taskEndTime": format_ts_to_iso(t_et),
                    "taskType": task_type,
                    "viewID": str(raw_req.get('viewID', 'YJ-1')),
                    "viewLon": float(raw_req.get('viewLon', 0.0)),
                    "viewLat": float(raw_req.get('viewLat', 0.0)),
                    "viewAlt": float(raw_req.get('viewAlt', 0.0)),
                    "realPointTime": real_time,
                    "realPointPos": real_pos,
                    "realPointVel": real_vel,
                    "predPointCnt": len(pred_pts),
                    "predPointTime": pred_times,
                    "predPointPos": pred_pts,
                    "predPointVel": pred_vels
                }
                mission_counter += 1

                dds_msg = make_dds_wrapper(
                    TOPIC_YJ_TASK_ASSIGN,
                    domain_key,
                    f"预警任务分配数据-SCID{scid}",
                    yj_assign_data
                )
                schedule_res["taskAssignList"].append(dds_msg)

        # ----------------------------------------------------------------------
        # 2. 构建单星任务动作指令 (sinsate_command.idl)
        # ----------------------------------------------------------------------
        for t in active_tasks:
            scid = int(t.get('scid', 1))
            roll_angle = float(t.get('rollAngle', 0.0))
            qw, qx, qy, qz = euler_to_quaternion(roll_angle, 0.0, 0.0)

            command_valid_data = {
                "SCID": scid,
                "imageSendTime": format_ts_to_iso(t['start_time']),
                "rollAngle": roll_angle,
                "pitchAngle": 0.0,
                "yawAngle": 0.0,
                "attiQuaterW": qw,
                "attiQuaterX": qx,
                "attiQuaterY": qy,
                "attiQuaterZ": qz
            }

            dds_cmd = make_dds_wrapper(
                TOPIC_SINSATE_COMMAND,
                "SINSATE",
                f"单星任务指令数据-SCID{scid}-{t['task_id']}",
                command_valid_data
            )
            schedule_res["commandList"].append(dds_cmd)

        # ----------------------------------------------------------------------
        # 3. 评测指标计算 (Metrics)
        # ----------------------------------------------------------------------
        total_targets = len(self.tgt_dic)
        observed_cnt = len(observed_tgt_ids)
        comp_rate = round(observed_cnt / total_targets, 4) if total_targets > 0 else 1.0

        req_start_ts = self.receive_time
        delays = [max(0.0, t['start_time'] - req_start_ts) for t in active_tasks]
        avg_resp_ms = int((sum(delays) / len(delays)) * 1000) if delays else 0

        # 综合得分 (100分制)
        score_comp = 50.0 * comp_rate
        score_time = 30.0 * max(0.0, 1.0 - (self.runtime_ms / 10000.0))
        score_resp = 20.0 * max(0.0, 1.0 - (avg_resp_ms / 3600000.0))
        overall_score = round(score_comp + score_time + score_resp, 2)

        schedule_res["metrics"] = {
            "missionMode": self.mission_mode,
            "completionRate": comp_rate,
            "avgResponseTime": avg_resp_ms,
            "assignedSatCount": len(sat_assigned_tasks),
            "commandCount": len(schedule_res["commandList"]),
            "schedulingDuration": self.runtime_ms,
            "overallScore": overall_score
        }

        return schedule_res

    def read_sol(self):
        """解析求解器输出文件"""
        self.task_info = {}
        self.tc_info = {}
        try:
            with open(self.solu_path, 'r', encoding='utf-8', errors='ignore') as f:
                for line in f.readlines():
                    line_str = line.strip()
                    if not line_str or line_str.startswith('#') or line_str.startswith('*'):
                        continue

                    var_name, val_num = None, None

                    # 1. 尝试匹配 XML/HTML 格式: <variable name="x_0" value="1"/>
                    xml_match = re.search(r'name=["\']([^"\']+)["\'].*?value=["\']([^"\']+)["\']', line_str)
                    if xml_match:
                        var_name = xml_match.group(1)
                        try:
                            val_num = float(xml_match.group(2))
                        except ValueError:
                            pass

                    # 2. 尝试匹配通用文本格式 (0 x_0 1.0 或 x_0 = 1.0)
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
                            if candidate_name.upper() in ['VAR', 'VARIABLE'] and len(parts) > 2:
                                candidate_name = parts[1]
                                candidate_val = parts[2]
                            try:
                                val_num = float(candidate_val)
                                var_name = candidate_name
                            except ValueError:
                                pass

                    if var_name is None or val_num is None:
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
