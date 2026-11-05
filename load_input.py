# ==============================================================================
# 文件名称: load_input.py
# 功能描述: 卫星任务规划系统的数据接入与预处理引擎 (完全基于星间 DDS 数据架构)
#          负责解析 DDS 规范的侦察需求 (zc_req) 与预警需求 (target_guide) 报文，
#          并融合卫星静态属性 (sate_static_config) 及候选时空窗口进行结构归一化。
# ==============================================================================

import re
import json
import time
from typing import Dict, Tuple, Any, List
from constants import (
    parse_time_to_ts,
    format_ts_to_iso,
    WARNING_PRIORITY_MULTIPLIER,
    OBSERVATION_DEFAULT_DURATION,
    DEFAULT_SAT_CONSTRAINTS
)

def to_scid(val: Any) -> int:
    """提取或映射出标准的 uint32 数值型 SCID"""
    if isinstance(val, int):
        return val
    s = str(val).strip()
    digits = re.findall(r'\d+', s)
    if digits:
        return int(digits[-1])
    return (abs(hash(s)) % 100000) + 1

class InputLoader:
    """
    DDS 标准输入数据预处理与加载器：
    完全遵循星间 DDS 数据架构 2026.10.16 规范。
    """
    def __init__(self, data: Dict[str, Any]) -> None:
        self.raw_data = data
        self.input_data: Dict[str, Any] = {}
        self.data_mapping: Dict[str, Any] = {
            "sat_task": {},
            "tgt_task": {},
            "sat_win": {},
            "sat_rt": {}
        }

    def model_input(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        """
        主入口函数：解析并构建规划数学模型所需的标准数据结构与映射
        """
        if not isinstance(self.raw_data, dict):
            raise ValueError("传入的 JSON 数据不合法，根对象必须为一个字典。")

        # 1. 解包 DDS 外壳 (若存在 dataname.validData 则展开)
        self.payload = self._unwrap_payload(self.raw_data)

        # 2. 识别任务模式：侦察 (zc) 或 预警 (yj)
        self.mission_mode = self._detect_mission_mode(self.payload)
        self.input_data['mission_mode'] = self.mission_mode

        # 3. 解析需求数据与目标定义
        self.load_demand()

        # 4. 解析卫星配置与能力约束 (支持 sate_static_config)
        self.load_satellite_config()

        # 5. 解析候选可见任务、数传窗口与星间路由
        self.load_meta_task_list()
        self.load_trans_window_list()
        self.load_routing_table_list()

        # 6. 构建求解器索引映射字典
        self.build_data_mapping()

        return self.input_data, self.data_mapping

    def _unwrap_payload(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """递归提取 DDS 报文中的有效数据有效载荷，自动解开 {'value': ...} 包装"""
        raw_valid = data
        if "dataname" in data and isinstance(data["dataname"], dict):
            raw_valid = data["dataname"].get("validData", {})

        merged = {}
        for k, v in raw_valid.items():
            if isinstance(v, dict) and "value" in v:
                merged[k] = v["value"]
            else:
                merged[k] = v

        for k in ["metaTaskList", "transWindowList", "routingTableList", "sateStaticConfig", "receiveTime"]:
            if k in data and k not in merged:
                merged[k] = data[k]
        return merged

    def _detect_mission_mode(self, payload: Dict[str, Any]) -> str:
        """
        自动判定业务类型：
        - yj (预警任务): 包含 taskType in ['Tracking', 'Staring'] 或 taskID 以 'REQ-YJ' 开头，或包含 viewID
        - zc (侦察任务): 包含 zcRequestId 或 pointTargetPos 或 pointTargetCnt 或 areaTargetCnt
        """
        if "taskType" in payload or "viewID" in payload or str(payload.get("taskID", "")).startswith("REQ-YJ"):
            return "yj"
        if "zcRequestId" in payload or "pointTargetPos" in payload or "areaTargetCnt" in payload or "zcReq" in payload:
            return "zc"
        # 默认按照侦察需求处理
        return "zc"

    def load_demand(self) -> None:
        """
        加载并标准化需求数据：
        - 对 zc 模式：解析 pointTargetPos 与 areaPotPos
        - 对 yj 模式：解析 Staring(凝视视场) 或 Tracking(轨迹点)
        """
        p = self.payload
        self.input_data['tgt_dic'] = {}
        self.input_data['demand_info'] = {}

        # 接收并记录规划触发时间
        raw_recv = p.get('timestamp') or p.get('receiveTime') or time.time()
        self.input_data['receive_time'] = parse_time_to_ts(raw_recv)

        if self.mission_mode == "zc":
            # ----------------- 侦察任务规范解析 (zc_req.idl) -----------------
            zc_req_id = str(p.get('zcRequestId', p.get('taskID', 'REQ-ZC-DEFAULT')))
            priority = int(p.get('priority', 1))
            st = parse_time_to_ts(p.get('taskStartTime', time.time()))
            et = parse_time_to_ts(p.get('taskEndTime', time.time() + 86400))

            self.input_data['demand_info'] = {
                'reqId': zc_req_id,
                'priority': priority,
                'startTime': st,
                'endTime': et,
                'sourceNode': int(p.get('sourceNode', 0)),
                'targetNode': int(p.get('targetNode', 1)),
                'raw_req': p
            }

            # 1. 解析点目标 (pointTargetPos)
            pt_list = p.get('pointTargetPos', [])
            pt_types = p.get('pointTargetType', [])
            pt_prios = p.get('pointTargetPriority', [])
            pt_payloads = p.get('pointTargetPayload', [])

            for i, pt in enumerate(pt_list):
                t_id = str(pt.get('targetId', f'PT-{i+1}'))
                t_prio = int(pt_prios[i]) if i < len(pt_prios) else priority
                t_type = str(pt_types[i]) if i < len(pt_types) else "Unknown"
                t_load = str(pt_payloads[i]) if i < len(pt_payloads) else "Panchromatic"

                self.input_data['tgt_dic'][t_id] = {
                    'targetId': t_id,
                    'type': 'point',
                    'category': t_type,
                    'priority': t_prio,
                    'repeat_count': 1,
                    'pos': pt,
                    'payload': t_load,
                    'reqId': zc_req_id
                }

            # 2. 解析区域目标 (areaPot1Pos ~ areaPot4Pos)
            area_pot1 = p.get('areaPot1Pos', [])
            area_pot2 = p.get('areaPot2Pos', [])
            area_pot3 = p.get('areaPot3Pos', [])
            area_pot4 = p.get('areaPot4Pos', [])
            area_types = p.get('areaTargetType', [])
            area_prios = p.get('areaTargetPriority', [])
            area_payloads = p.get('areaTargetPayload', [])

            for i, pot1 in enumerate(area_pot1):
                a_id = str(pot1.get('areaId', f'AT-{i+1}'))
                a_prio = int(area_prios[i]) if i < len(area_prios) else priority
                a_type = str(area_types[i]) if i < len(area_types) else "Unknown"
                a_load = str(area_payloads[i]) if i < len(area_payloads) else "Panchromatic"

                pot2 = area_pot2[i] if i < len(area_pot2) else pot1
                pot3 = area_pot3[i] if i < len(area_pot3) else pot1
                pot4 = area_pot4[i] if i < len(area_pot4) else pot1

                self.input_data['tgt_dic'][a_id] = {
                    'targetId': a_id,
                    'type': 'area',
                    'category': a_type,
                    'priority': a_prio,
                    'repeat_count': 1,
                    'vertices': [pot1, pot2, pot3, pot4],
                    'payload': a_load,
                    'reqId': zc_req_id
                }

        else:
            # ----------------- 预警任务规范解析 (target_guide.idl) -----------------
            task_id = str(p.get('taskID', 'REQ-YJ-DEFAULT'))
            priority = int(p.get('priority', 1))
            task_type = str(p.get('taskType', 'Staring'))
            st = parse_time_to_ts(p.get('taskStartTime', time.time()))
            et = parse_time_to_ts(p.get('taskEndTime', time.time() + 3600))

            self.input_data['demand_info'] = {
                'reqId': task_id,
                'taskType': task_type,
                'priority': priority,
                'startTime': st,
                'endTime': et,
                'sourceNode': int(p.get('sourceNode', 0)),
                'targetNode': int(p.get('targetNode', 1)),
                'raw_req': p
            }

            if task_type == 'Staring':
                # 凝视视场中心
                view_id = str(p.get('viewID', 'YJ-STARING-1'))
                self.input_data['tgt_dic'][view_id] = {
                    'targetId': view_id,
                    'type': 'staring',
                    'priority': priority,
                    'repeat_count': 1,
                    'viewLon': float(p.get('viewLon', 0.0)),
                    'viewLat': float(p.get('viewLat', 0.0)),
                    'viewAlt': float(p.get('viewAlt', 0.0)),
                    'reqId': task_id,
                    'is_emergency': True
                }
            else:
                # 连续跟踪动态航迹点序列
                track_id = str(p.get('viewID', 'YJ-TRACKING-1'))
                self.input_data['tgt_dic'][track_id] = {
                    'targetId': track_id,
                    'type': 'tracking',
                    'priority': priority,
                    'repeat_count': 1,
                    'pointCnt': int(p.get('pointCnt', len(p.get('pointPosList', [])))),
                    'pointTime': p.get('pointTime', []),
                    'pointPosList': p.get('pointPosList', []),
                    'pointVelList': p.get('pointVelList', []),
                    'reqId': task_id,
                    'is_emergency': True
                }

    def load_satellite_config(self) -> None:
        """
        加载卫星物理参数与能力配置：
        优先支持 DDS sate_static_config 规范，自动映射 SCID 与速率约束。
        """
        p = self.payload
        self.input_data['sat_dic'] = {}
        self.input_data['scid_mapping'] = {} # str_id -> scid_int
        self.input_data['reverse_scid'] = {} # scid_int -> str_id

        # 检查是否提供了 sateStaticConfig
        static_configs = p.get('sateStaticConfig', p.get('staticConfigs', []))
        if isinstance(static_configs, dict):
            static_configs = [static_configs]

        # 兜底默认值
        default_sat_param = {
            "tc_duration": 1,
            "process_rate": 0.5,
            "transmission_rate": 2.0,
            "max_uptime": 60,
            "energy_capacity": 1000.0,
            "max_roll": 45.0,
            "slew_rate": 3.0
        }

        # 默认卫星能力分级配置
        cons = DEFAULT_SAT_CONSTRAINTS

        if static_configs:
            for cfg in static_configs:
                scid = int(cfg.get('SCID', 1))
                sat_key = f"SAT_{scid}"
                self.input_data['scid_mapping'][sat_key] = scid
                self.input_data['reverse_scid'][scid] = sat_key

                roll_range = float(cfg.get('rollRange', cfg.get('maxRollAngle', 45.0)))
                slew_rate = float(cfg.get('attManeuverRate', 3.0))
                downlink_rate = float(cfg.get('maxGroundDownlinkRate', 2.0))
                energy_cap = float(cfg.get('onboardEnergyCapacity', 1000.0))

                self.input_data['sat_dic'][sat_key] = {
                    "tc_duration": 1,
                    "process_rate": 0.5,
                    "transmission_rate": downlink_rate if downlink_rate > 0 else 2.0,
                    "max_uptime": 60,
                    "energy_capacity": energy_cap,
                    "max_roll": roll_range,
                    "slew_rate": slew_rate,
                    "scid": scid
                }
        else:
            # 若未提供 sateStaticConfig，从 metaTaskList 提取涉及的全部卫星
            meta_tasks = p.get('metaTaskList', [])
            sat_ids = list(set(str(item.get('satelliteId', item.get('SCID', 1))) for item in meta_tasks if 'satelliteId' in item or 'SCID' in item))
            if not sat_ids:
                sat_ids = ["SAT_1"]

            for sid in sat_ids:
                scid = to_scid(sid)
                sat_key = f"SAT_{scid}"
                self.input_data['scid_mapping'][sid] = scid
                self.input_data['scid_mapping'][sat_key] = scid
                self.input_data['reverse_scid'][scid] = sat_key

                base_conf = cons.get('middle' if scid == 1 else 'low', default_sat_param)
                conf = dict(default_sat_param)
                conf.update(base_conf)
                conf["scid"] = scid
                self.input_data['sat_dic'][sat_key] = conf

        # 主星标识 (默认 SCID=1)
        self.input_data['cluster_info'] = {
            'main_sat': list(self.input_data['sat_dic'].keys())[0],
            'sat_set': list(self.input_data['sat_dic'].keys())
        }

    def load_meta_task_list(self) -> None:
        """
        加载候选观测任务元列表 (metaTaskList)，将时间转换为 Unix 浮点秒
        """
        task_src = self.payload.get('metaTaskList', [])
        self.input_data['task_list'] = []
        self.input_data['sat_earliest_end'] = {}
        self.input_data['sat_endest_start'] = {}

        # 若外部未直接提供 metaTaskList，而只给了一个点/面目标，自动基于需求时段生成保底任务弧段
        if not task_src:
            demand = self.input_data.get('demand_info', {})
            st = demand.get('startTime', time.time())
            et = demand.get('endTime', st + 3600)
            for s_key in self.input_data['sat_dic']:
                for t_id in self.input_data['tgt_dic']:
                    task_src.append({
                        "taskId": f"MT-{s_key}-{t_id}",
                        "targetId": t_id,
                        "satelliteId": s_key,
                        "visStartTime": format_ts_to_iso(st + 60),
                        "visEndTime": format_ts_to_iso(st + 60 + OBSERVATION_DEFAULT_DURATION),
                        "rollAngle": 5.0
                    })

        for count, item in enumerate(task_src):
            raw_sat = item.get('satelliteId', item.get('SCID', 1))
            scid = to_scid(raw_sat)
            sat_key = f"SAT_{scid}"

            item['sat_id'] = sat_key
            item['scid'] = scid
            item['satelliteId'] = sat_key
            item['task_id'] = item.get('taskId', item.get('task_id', f"task_{count}"))
            item['tgt_id'] = item.get('targetId', item.get('tgt_id'))

            st = parse_time_to_ts(item.get('visStartTime', item.get('startTime')))
            et = parse_time_to_ts(item.get('visEndTime', item.get('endTime')))

            item['start_time'] = st
            item['end_time'] = et
            item['duration'] = max(1.0, et - st)
            item['task_idx'] = count
            item['rollAngle'] = float(item.get('rollAngle', 0.0))

            # 统计各卫星的时间跨度边界
            if sat_key in self.input_data['sat_earliest_end']:
                self.input_data['sat_earliest_end'][sat_key] = min(self.input_data['sat_earliest_end'][sat_key], et)
                self.input_data['sat_endest_start'][sat_key] = max(self.input_data['sat_endest_start'][sat_key], st)
            else:
                self.input_data['sat_earliest_end'][sat_key] = et
                self.input_data['sat_endest_start'][sat_key] = st

            self.input_data['task_list'].append(item)

    def load_trans_window_list(self) -> None:
        """加载星地数传窗口列表 (transWindowList)"""
        win_list = self.payload.get('transWindowList', [])
        for w in win_list:
            raw_b = w.get('sidBId', w.get('satelliteId', 1))
            scid = to_scid(raw_b)
            w['sat_id'] = f"SAT_{scid}"
            w['scid'] = scid
            w['start_time'] = parse_time_to_ts(w.get('startTime'))
            w['end_time'] = parse_time_to_ts(w.get('endTime'))
        self.input_data['win_list'] = win_list

    def load_routing_table_list(self) -> None:
        """加载星间路由表 (routingTableList)"""
        rt_list = self.payload.get('routingTableList', [])
        for rt in rt_list:
            raw_s1 = rt.get('satelliteId', 1)
            raw_s2 = rt.get('desSatellited', rt.get('desSatelliteId', 2))
            rt['sat1_key'] = f"SAT_{to_scid(raw_s1)}"
            rt['sat2_key'] = f"SAT_{to_scid(raw_s2)}"
            rt['start_time'] = parse_time_to_ts(rt.get('startTime'))
            rt['end_time'] = parse_time_to_ts(rt.get('endTime'))
        self.input_data['rt_list'] = rt_list

    def build_data_mapping(self) -> None:
        """构建求解器快速索引映射表"""
        self.data_mapping = {
            "sat_task": {},
            "tgt_task": {},
            "sat_win": {},
            "sat_rt": {}
        }

        # 1. 映射任务
        for item in self.input_data.get('task_list', []):
            sat_id = item['sat_id']
            tgt_id = item['tgt_id']
            idx = item['task_idx']
            self.data_mapping['sat_task'].setdefault(sat_id, []).append(idx)
            self.data_mapping['tgt_task'].setdefault(tgt_id, []).append(idx)

        # 2. 映射数传窗口
        for i, w in enumerate(self.input_data.get('win_list', [])):
            sid = w.get('sat_id')
            if sid:
                self.data_mapping['sat_win'].setdefault(sid, []).append(i)

        # 3. 映射星间通信弧段
        for i, rt in enumerate(self.input_data.get('rt_list', [])):
            s1 = rt.get('sat1_key')
            s2 = rt.get('sat2_key')
            if s1:
                self.data_mapping['sat_rt'].setdefault(s1, []).append(i)
            if s2:
                self.data_mapping['sat_rt'].setdefault(s2, []).append(i)