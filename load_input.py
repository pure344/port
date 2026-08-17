# ==============================================================================
# 文件名称: load_input.py
# 功能描述: 卫星任务规划系统的数据接入与预处理引擎
# 
# 【外部接口协议与内部映射架构文档】
# 
# 1. 外部接口协议 (JSON 契约):
#    1.1 需求输入 (requests / requestsList):
#        - 代表用户的观测意图。
#        - 关键字段: reqId (需求唯一标识), targetList (需要拍摄的地面目标点集合),
#          reqTimes (要求对该目标进行重访的次数)。
#        - 控制标志: isOnboardProcess (是否星上直接处理，缩短时延),
#          isRawDownload (是否下传原始观测数据)。
#    
#    1.2 卫星与目标元数据 (meta_task_list):
#        - 描述所有物理可用卫星的基本参数。
#        - 关键字段: sat_id (卫星标识), sat_type (卫星类型，如光学/雷达),
#          power_capacity (电池容量), memory_capacity (存储容量)。
# 
#    1.3 观测时间窗口 (task_list):
#        - 经过轨道动力学预先推算出的，卫星对地可见的物理窗口集合。
#        - 每个窗口绑定特定的 sat_id 与 target_id。
#        - 算法的决策核心即是从这些大量重叠的窗口中挑选出一个无冲突的最优子集。
# 
#    1.4 数据下传与测控窗口 (trans_window_list / control_window_list):
#        - 描述卫星与各个地面站 (Ground Station) 之间的可见情况。
#        - 分为下传窗口 (dt_start, dt_end) 用于数据回传，
#          和测控窗口 (tc_start, tc_end) 用于上行指令注入。
# 
#    1.5 应急任务注入 (emergency_info):
#        - 用于处理突发的高优先级任务 (如灾害应急)。
#        - 具备打断当前常规任务队列的能力。
# 
# 2. 内部映射架构 (ID Mapping):
#    - 由于 MILP 求解器通常要求决策变量下标为连续整数 (0, 1, 2...)，
#      而外部输入的各类 ID 是长字符串 (如 UUID 或特定的命名规则)。
#    - 本模块通过构建四组核心映射字典来建立内外桥梁：
#      1) sat_task: { sat_id: [task_idx_1, task_idx_2, ...] }，归拢某颗卫星的所有观测任务索引。
#      2) tgt_task: { target_id: [task_idx_1, ...] }，归拢某个目标的所有候选观测任务索引。
#      3) sat_win: { sat_id: [win_idx_1, ...] }，归拢某颗卫星的下传窗口索引。
#      4) sat_rt: { sat_id: [rt_idx_1, ...] }，归拢某颗卫星的测控窗口索引。
# 
# 3. 容错与降级设计:
#    - 字段容错: 对 requests 和 requestsList 兼容，保障上下游 API 变动时的鲁棒性。
#    - 默认值补偿: 当输入 JSON 缺失某些物理参数 (如电量、姿态机动耗时) 时，
#      自动填充经验默认值，避免算法崩溃。
# ==============================================================================
import json
from datetime import datetime

from typing import Dict, Tuple, Any

class InputLoader:
    """
    输入数据预处理与加载器：
    负责将外部输入的原始 JSON 数据解析、格式转换，并构建算法模型所需的标准数据与映射关系。
    """
    def __init__(self, data: Dict[str, Any]) -> None:
        """
        初始化输入数据加载器。

        Args:
            data (Dict[str, Any]): 从外部接口接收的原始 JSON 解析后的字典数据。
        """
        self.data = data
        self.input_data: Dict[str, Any] = {}
        self.data_mapping: Dict[str, Any] = {
            "sat_task": {},
            "tgt_task": {},
            "sat_win": {},
            "sat_rt": {}
        }

    def model_input(self):
        """
        主入口函数：按顺序调用各个模块化加载函数，完成所有字段与映射的加载。
        """
        if not isinstance(self.data, dict):
            raise ValueError("传入的 JSON 格式不正确，必须为一个标准的字典对象。")

        # 核心前置校验：如果没有输入 requests 或 metaTaskList，则无业务可做，直接报错提示
        reqs = self.data.get('requests', self.data.get('requestsList', []))
        if not reqs:
            raise ValueError("异常数据：输入的请求单(requests/requestsList)为空或缺失，请检查！")
            
        meta_tasks = self.data.get('metaTaskList', [])
        if not meta_tasks:
            raise ValueError("异常数据：候选任务列表(metaTaskList)为空或缺失，无法进行规划！")

        # 1. 独立解析各个 Key-Value 数据项
        try:
            self.load_receive_time()
            self.load_requests()
            self.load_requests_supply()
            self.load_meta_task_list()
            self.load_trans_window_list()
            self.load_routing_table_list()
            self.load_emergency_info()
        except ValueError as e:
            # 捕获我们在特定加载函数中抛出的友好异常
            raise e
        except Exception as e:
            raise ValueError(f"异常数据：解析输入数据时发生未知错误，可能是字段类型或格式不合法 ({str(e)})")

        # 2. 统一构建数据映射关系
        self.build_data_mapping()

        return self.input_data, self.data_mapping

    def load_receive_time(self):
        """
        【读取 Key: receiveTime】
        功能：解析系统接收需求的时间戳（字符串格式转为标准字符串，若无则取当前时间）。
        """
        self.input_data['receive_time'] = self.data.get('receiveTime', datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

    def load_requests(self):
        """
        【读取 Key: requests / requestsList】
        功能：解析用户需求单列表 (List/Dict)，优先读取 requests，降级兼容 requestsList。
             提取每个需求单中的重访次数 reqTimes、目标列表 targetList、
             元数据以及需求控制标志位 (isRawDownload/isRawDownlink/isOnboardProcess/isOnboard)，
             存入 input_data['tgt_dic']、input_data['requests'] 和 input_data['flag_list']。
        """
        reqs = self.data.get('requests', self.data.get('requestsList', []))
        if isinstance(reqs, dict):
            reqs = [reqs]

        self.input_data['tgt_dic'] = {}
        self.input_data['requests'] = []

        # 从首个需求单提取全局规划模式控制标志位
        first_req = reqs[0] if len(reqs) > 0 else {}
        self.input_data['flag_list'] = {
            'is_raw_data_download': first_req.get('isRawDownload', first_req.get('isRawDownlink', False)),
            'is_sub_process': first_req.get('isOnboardProcess', True),
            'isdistribute': first_req.get('isOnboard', False)
        }

        for req in reqs:
            req_times = req.get('reqTimes', 1)
            target_list = req.get('targetList', [])

            # 存储需求单元数据及专属控制标志
            self.input_data['requests'].append({
                'zcRequestId': req.get('zcRequestId', ''),
                'resource': req.get('resource', ''),
                'priority': req.get('priority', 1),
                'isRawDownload': req.get('isRawDownload', req.get('isRawDownlink', False)),
                'isOnboardProcess': req.get('isOnboardProcess', True),
                'isOnboard': req.get('isOnboard', False),
                'targetList': target_list
            })

            # 解析目标列表及重复观测需求
            for tgt in target_list:
                target_id = tgt.get('targetId')
                if target_id:
                    self.input_data['tgt_dic'][target_id] = {
                        'repeat_count': req_times,
                        'priority': tgt.get('yxj', 1)
                    }

    def load_requests_supply(self):
        """
        【读取 Key: requestsSupply】
        功能：解析集群卫星架构配置 (主星 mainSat、卫星集合 satelliteSet)，并融合 config/constraint.json 
             配置文件生成每颗卫星的硬件参数字典 (包含处理速率、传输速率、能耗上限等)。
        """
        req_supply = self.data.get('requestsSupply', {})
        if isinstance(req_supply, list):
            first_item = req_supply[0] if len(req_supply) > 0 and isinstance(req_supply[0], dict) else {}
            cluster_cfg = first_item.get('clusterConfig', {})
            main_sat = cluster_cfg.get('mainSat', first_item.get('mainSat', 'SAT_01'))
            sat_set = cluster_cfg.get('satelliteSet', first_item.get('satelliteSet', []))
        else:
            cluster_cfg = req_supply.get('clusterConfig', {})
            main_sat = cluster_cfg.get('mainSat', req_supply.get('mainSat', 'SAT_01'))
            sat_set = cluster_cfg.get('satelliteSet', req_supply.get('satelliteSet', []))

        # 若未配置 satelliteSet，从候选任务列表 metaTaskList 自动提取出现的全部卫星 ID
        if not sat_set:
            meta_tasks = self.data.get('metaTaskList', [])
            sat_set = list(set(item.get('satelliteId', item.get('sat_id')) for item in meta_tasks if item.get('satelliteId') or item.get('sat_id')))
            if not sat_set:
                sat_set = [main_sat]

        if main_sat not in sat_set and len(sat_set) > 0:
            main_sat = sat_set[0]

        self.input_data['cluster_info'] = {
            'main_sat': main_sat,
            'sat_set': sat_set
        }

        # 融合 constraint.json 配置文件
        try:
            cons = json.load(open('./config/constraint.json'))
        except Exception:
            cons = {
                "middle": {"tc_duration": 1, "process_rate": 0.5, "transmission_rate": 4, "max_uptime": 60, "energy_capacity": 600},
                "low": {"tc_duration": 1, "process_rate": 1, "transmission_rate": 2, "max_uptime": 60, "energy_capacity": 600}
            }

        self.input_data['sat_dic'] = {}
        for sat_id in sat_set:
            self.input_data['sat_dic'][sat_id] = cons.get('low', {})
        if main_sat in self.input_data['sat_dic']:
            self.input_data['sat_dic'][main_sat] = cons.get('middle', cons.get('low', {}))

    def load_meta_task_list(self):
        """
        【读取 Key: metaTaskList】
        功能：解析候选观测任务列表，将可见时间 (visStartTime/visEndTime) 转换为 UNIX 浮点时间戳，
             计算任务时长 duration，并统计每颗卫星的时间边界 (earliest_end / endest_start)。
        """
        self.input_data['task_list'] = []
        self.input_data['sat_earliest_end'] = {}
        self.input_data['sat_endest_start'] = {}

        task_src = self.data.get('metaTaskList', [])
        for count, item in enumerate(task_src):
            sat_id = item.get('satelliteId', item.get('sat_id'))
            item['sat_id'] = sat_id
            item['task_id'] = item.get('taskId', item.get('task_id'))
            if not item['task_id']:
                item['task_id'] = f"auto_task_{count}"
            item['tgt_id'] = item.get('targetId', item.get('tgt_id'))

            st_str = item.get('visStartTime', item.get('startTime'))
            et_str = item.get('visEndTime', item.get('endTime'))

            st = datetime.strptime(st_str, "%Y-%m-%d %H:%M:%S").timestamp()
            et = datetime.strptime(et_str, "%Y-%m-%d %H:%M:%S").timestamp()

            item['start_time'] = st
            item['end_time'] = et
            item['duration'] = max(1.0, et - st)
            item['task_idx'] = count

            # 统计卫星的时间区间边界
            if sat_id in self.input_data['sat_earliest_end']:
                self.input_data['sat_earliest_end'][sat_id] = min(self.input_data['sat_earliest_end'][sat_id], et)
                self.input_data['sat_endest_start'][sat_id] = max(self.input_data['sat_endest_start'][sat_id], st)
            else:
                self.input_data['sat_earliest_end'][sat_id] = et
                self.input_data['sat_endest_start'][sat_id] = st

            self.input_data['task_list'].append(item)

    def load_trans_window_list(self):
        """
        【读取 Key: transWindowList】
        功能：解析星地数传/接收窗口列表，将起止时间转换为 UNIX 浮点时间戳并存入 input_data['win_list']。
        """
        win_list = self.data.get('transWindowList', [])
        for w in win_list:
            w['start_time'] = datetime.strptime(w['startTime'], "%Y-%m-%d %H:%M:%S").timestamp()
            w['end_time'] = datetime.strptime(w['endTime'], "%Y-%m-%d %H:%M:%S").timestamp()
        self.input_data['win_list'] = win_list

    def load_routing_table_list(self):
        """
        【读取 Key: routingTableList】
        功能：解析星间路由表列表，将通信弧段起止时间转换为 UNIX 浮点时间戳并存入 input_data['rt_list']。
        """
        rt_list = self.data.get('routingTableList', [])
        for rt in rt_list:
            rt['start_time'] = datetime.strptime(rt['startTime'], "%Y-%m-%d %H:%M:%S").timestamp()
            rt['end_time'] = datetime.strptime(rt['endTime'], "%Y-%m-%d %H:%M:%S").timestamp()
        self.input_data['rt_list'] = rt_list

    def load_emergency_info(self):
        """
        【读取 Key: emergencyStatus / emergencyTasks / requestsEmergencyIds / requestEmergencyIds】
        功能：解析应急响应标志位、历史冲突任务列表以及应急需求单下标数组/单号数组，
             提取出所有应急需求对应的目标 ID 集合存入 input_data['emergency_tgt_ids']。
        """
        self.input_data['emergency_status'] = self.data.get('emergencyStatus', False) or False

        raw_emergency = self.data.get('emergencyTasks') or self.data.get('emergencyTask') or []
        if not isinstance(raw_emergency, list):
            raw_emergency = []
        self.input_data['emergency_tasks'] = [t for t in raw_emergency if isinstance(t, dict) and t.get('taskId')]
        
        em_ids = self.data.get('requestsEmergencyIds') or self.data.get('requestEmergencyIds') or []
        if not isinstance(em_ids, list):
            em_ids = [em_ids] if em_ids is not None else []
        self.input_data['requests_emergency_ids'] = em_ids

        # 提取属于应急需求的所有目标 ID 集合
        self.input_data['emergency_tgt_ids'] = set()
        reqs = self.input_data.get('requests', self.data.get('requests', self.data.get('requestsList', [])))
        if isinstance(reqs, dict):
            reqs = [reqs]

        for em_idx in self.input_data['requests_emergency_ids']:
            if isinstance(em_idx, int) and 0 <= em_idx < len(reqs):
                target_list = reqs[em_idx].get('targetList', [])
                for tgt in target_list:
                    if isinstance(tgt, dict) and tgt.get('targetId'):
                        self.input_data['emergency_tgt_ids'].add(tgt['targetId'])
            elif isinstance(em_idx, str):
                # 支持通过需求单号字符串 (如 "ZC20260728010") 进行匹配
                for req in reqs:
                    if req.get('zcRequestId') == em_idx:
                        for tgt in req.get('targetList', []):
                            if isinstance(tgt, dict) and tgt.get('targetId'):
                                self.input_data['emergency_tgt_ids'].add(tgt['targetId'])

    def load_flag_list(self):
        """
        【读取算法标志位】
        功能：保留兼容降级入口。
        """
        pass

    def build_data_mapping(self):
        """
        【统一构建数据映射字典】
        功能：集中构建算法模型求解所需的所有多对多/一对多索引表：
             1. sat_task: 卫星 ID -> 候选任务索引列表
             2. tgt_task: 目标 ID -> 候选任务索引列表
             3. sat_win: 卫星 ID -> 数传窗口索引列表
             4. sat_rt: 卫星 ID -> 星间路由弧段索引列表
        """
        self.data_mapping = {
            "sat_task": {},
            "tgt_task": {},
            "sat_win": {},
            "sat_rt": {}
        }

        # 1. 构建 sat_task 与 tgt_task 映射
        for item in self.input_data.get('task_list', []):
            sat_id = item['sat_id']
            tgt_id = item['tgt_id']
            idx = item['task_idx']
            self.data_mapping['sat_task'].setdefault(sat_id, []).append(idx)
            self.data_mapping['tgt_task'].setdefault(tgt_id, []).append(idx)

        # 2. 构建 sat_win 映射
        for i, w in enumerate(self.input_data.get('win_list', [])):
            sid = w.get('sidBId', w.get('sidAId'))
            if sid:
                self.data_mapping['sat_win'].setdefault(sid, []).append(i)

        # 3. 构建 sat_rt 映射
        for i, rt in enumerate(self.input_data.get('rt_list', [])):
            sat1 = rt.get('satelliteId')
            sat2 = rt.get('desSatellited', rt.get('desSatelliteId'))
            if sat1:
                self.data_mapping['sat_rt'].setdefault(sat1, []).append(i)
            if sat2:
                self.data_mapping['sat_rt'].setdefault(sat2, []).append(i)