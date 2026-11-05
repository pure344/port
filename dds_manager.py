# ==============================================================================
# 文件名称: dds_manager.py
# 功能描述: 卫星任务规划系统 DDS 核心通信管理模块 (基于 Eclipse CycloneDDS)
#          严格按照《星间DDS数据架构》实现侦察与预警数据域的事件驱动发布订阅。
#          包含完整的 IDL 结构映射、多域 Participant 管理与规划流水线调度。
# ==============================================================================

import os
import sys
import time
import logging
import threading
from typing import List, Dict, Any, Optional, Callable
from dataclasses import dataclass

# CycloneDDS 核心接口与数据类型
from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.core import Qos, Policy
from cyclonedds.util import duration
from cyclonedds.idl import IdlStruct, IdlEnum
from cyclonedds.idl.types import sequence, uint32, uint8, float64, float32

# 引入项目常量与算法模块
from constants import (
    DDS_DOMAIN_SINSATE,
    DDS_DOMAIN_RECON,
    DDS_DOMAIN_WARNING,
    TOPIC_ZC_REQ,
    TOPIC_ZC_TASK_ASSIGN,
    TOPIC_ZC_TASK_RESPONSE,
    TOPIC_TARGET_GUIDE,
    TOPIC_YJ_TASK_ASSIGN,
    TOPIC_YJ_TASK_RESPONSE,
    TOPIC_SINSATE_COMMAND,
    format_ts_to_iso
)
from run_algo import run

logger = logging.getLogger("DDSNodeManager")
if not logger.handlers:
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter('[%(asctime)s] [%(levelname)s] [DDS] %(message)s'))
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)

# ==============================================================================
# 1. DDS IDL 数据结构映射 (严格依照 星间DDS数据架构1016/IDL)
# ==============================================================================

# ----------------- 侦察任务 IDL (zc_req.idl, zc_task_assign.idl) -----------------
class PointTargetType(IdlEnum):
    POINT_CIVILSHIP = 0        # Civilship
    POINT_CARRIER = 1          # Carrier
    POINT_MISSILE_LAUNCHER = 2  # Missile_launcher
    POINT_UNKNOWN = 3          # Unknown

class PayloadType(IdlEnum):
    PAYLOAD_PANCHROMATIC = 0   # Panchromatic
    PAYLOAD_INFRARED = 1       # Infrared
    PAYLOAD_MULTISPECTRAL = 2  # Multispectral
    PAYLOAD_HYPERSPECTRAL = 3  # Hyperspectral
    PAYLOAD_SAR = 4            # SAR
    PAYLOAD_ELECTRONIC = 5     # Electronic
    PAYLOAD_RADAR = 6          # Radar

class AreaTargetType(IdlEnum):
    AREA_ISLAND = 0            # Island
    AREA_PORT = 1              # Port
    AREA_CITY = 2              # City
    AREA_UNKNOWN = 3           # Unknown

@dataclass
class PointTargetPosition(IdlStruct):
    targetId: str
    lon: float64
    lat: float64
    alt: float32

@dataclass
class AreaVertexPosition(IdlStruct):
    areaId: str
    lon: float64
    lat: float64
    alt: float32

@dataclass
class ZcReq(IdlStruct):
    """侦察任务需求报文 (对应 zc_req.idl)"""
    sourceNode: uint32
    targetNode: uint32
    timestamp: str
    zcRequestId: str
    priority: uint8
    taskStartTime: str
    taskEndTime: str
    pointTargetCnt: uint32
    pointTargetPos: sequence[PointTargetPosition]
    pointTargetType: sequence[PointTargetType]
    pointTargetPriority: sequence[uint8]
    pointTargetPayload: sequence[PayloadType]
    areaTargetCnt: uint32
    areaPot1Pos: sequence[AreaVertexPosition]
    areaPot2Pos: sequence[AreaVertexPosition]
    areaPot3Pos: sequence[AreaVertexPosition]
    areaPot4Pos: sequence[AreaVertexPosition]
    areaTargetType: sequence[AreaTargetType]
    areaTargetPriority: sequence[uint8]
    areaTargetPayload: sequence[PayloadType]

@dataclass
class ZcTaskAssign(IdlStruct):
    """侦察任务分配报文 (对应 zc_task_assign.idl)"""
    execNode: uint32
    missionID: uint32
    taskAssignTime: str
    taskStartTime: str
    taskEndTime: str
    pointTargetCnt: uint32
    pointTargetPos: sequence[PointTargetPosition]
    pointTargetType: sequence[PointTargetType]
    pointTargetPriority: sequence[uint8]
    pointTargetPayload: sequence[PayloadType]
    areaTargetCnt: uint32
    areaPot1Pos: sequence[AreaVertexPosition]
    areaPot2Pos: sequence[AreaVertexPosition]
    areaPot3Pos: sequence[AreaVertexPosition]
    areaPot4Pos: sequence[AreaVertexPosition]
    areaTargetType: sequence[AreaTargetType]
    areaTargetPriority: sequence[uint8]
    areaTargetPayload: sequence[PayloadType]

# ----------------- 预警任务 IDL (target_guide.idl, yj_task_assign.idl) -----------------
class YjTaskType(IdlEnum):
    TRACKING = 0               # 跟踪
    STARING = 1                # 凝视

@dataclass
class GeoPosition(IdlStruct):
    lon: float64
    lat: float64
    alt: float32

@dataclass
class GeoVelocity(IdlStruct):
    east: float64
    north: float64
    up: float64

@dataclass
class TargetGuide(IdlStruct):
    """预警目标引导报文 (对应 target_guide.idl)"""
    sourceNode: uint32
    targetNode: uint32
    timestamp: str
    taskID: str
    priority: uint8
    taskType: YjTaskType
    taskStartTime: str
    taskEndTime: str
    viewID: str
    viewLon: float64
    viewLat: float64
    viewAlt: float32
    pointCnt: uint32
    pointTime: sequence[str]
    pointPosList: sequence[GeoPosition]
    pointVelList: sequence[GeoVelocity]

@dataclass
class YjTaskAssign(IdlStruct):
    """预警任务分配报文 (对应 yj_task_assign.idl)"""
    execNode: uint32
    missionID: uint32
    taskAssignTime: str
    taskStartTime: str
    taskEndTime: str
    taskType: YjTaskType
    viewID: str
    viewLon: float64
    viewLat: float64
    viewAlt: float32
    realPointTime: str
    realPointPos: GeoPosition
    realPointVel: GeoVelocity
    predPointCnt: uint8
    predPointTime: sequence[str]
    predPointPos: sequence[GeoPosition]
    predPointVel: sequence[GeoVelocity]

# ----------------- 单星动作指令 IDL (sinsate_command.idl) -----------------
@dataclass
class SinsateCommand(IdlStruct):
    """单星任务动作指令 (对应 sinsate_command.idl)"""
    SCID: uint32
    imageSendTime: str
    rollAngle: float64
    pitchAngle: float64
    yawAngle: float64
    attiQuaterW: float32
    attiQuaterX: float32
    attiQuaterY: float32
    attiQuaterZ: float32

# ----------------- 规划应答通用 IDL -----------------
@dataclass
class TaskResponse(IdlStruct):
    """任务应答状态"""
    sourceNode: uint32
    targetNode: uint32
    timestamp: str
    taskID: str
    status: uint8            # 0: 规划成功, 1: 拒执/无可行解, 2: 异常
    message: str

# ==============================================================================
# 2. DDS 通信与调度核心管理器
# ==============================================================================
class DDSNodeManager:
    """
    卫星任务规划系统 DDS 节点管理器：
    负责加入指定 DDS 域、创建发布订阅通道，并在收到需求时自动触发 MILP 规划求解。
    """
    def __init__(self, work_dir: str = None, unified_domain: Optional[int] = None):
        self.work_dir = work_dir or os.path.dirname(os.path.abspath(__file__))
        self.running = False
        self.threads: List[threading.Thread] = []
        self.mission_counter = 1

        # 支持单一测试域或标准多域划分 (环境变量 DDS_UNIFIED_DOMAIN 优先)
        env_domain = os.environ.get('DDS_UNIFIED_DOMAIN')
        if unified_domain is not None:
            self.domain_recon = unified_domain
            self.domain_warning = unified_domain
            self.domain_sinsate = unified_domain
        elif env_domain is not None:
            dom = int(env_domain)
            self.domain_recon = dom
            self.domain_warning = dom
            self.domain_sinsate = dom
        else:
            self.domain_recon = DDS_DOMAIN_RECON      # 4 (侦察任务域)
            self.domain_warning = DDS_DOMAIN_WARNING  # 5 (预警任务域)
            self.domain_sinsate = DDS_DOMAIN_SINSATE  # 1 (单星数据域)

        # 可靠传输 QoS 策略
        self.qos_reliable = Qos(Policy.Reliability.Reliable(max_blocking_time=duration(seconds=1)))

        logger.info("正在初始化 CycloneDDS 参与者节点...")
        logger.info(f"  * 侦察任务域 (RECON): Domain ID {self.domain_recon}")
        logger.info(f"  * 预警任务域 (WARNING): Domain ID {self.domain_warning}")
        logger.info(f"  * 单星数据域 (SINSATE): Domain ID {self.domain_sinsate}")

        # 1. 创建 DomainParticipant
        self.participants: Dict[int, DomainParticipant] = {}
        for d in {self.domain_recon, self.domain_warning, self.domain_sinsate}:
            self.participants[d] = DomainParticipant(d)

        self.part_recon = self.participants[self.domain_recon]
        self.part_warning = self.participants[self.domain_warning]
        self.part_sinsate = self.participants[self.domain_sinsate]

        # 2. 创建 DDS 主题 (Topic)
        # 侦察域
        self.topic_zc_req = Topic(self.part_recon, TOPIC_ZC_REQ, ZcReq, qos=self.qos_reliable)
        self.topic_zc_assign = Topic(self.part_recon, TOPIC_ZC_TASK_ASSIGN, ZcTaskAssign, qos=self.qos_reliable)
        self.topic_zc_resp = Topic(self.part_recon, TOPIC_ZC_TASK_RESPONSE, TaskResponse, qos=self.qos_reliable)

        # 预警域
        self.topic_yj_guide = Topic(self.part_warning, TOPIC_TARGET_GUIDE, TargetGuide, qos=self.qos_reliable)
        self.topic_yj_assign = Topic(self.part_warning, TOPIC_YJ_TASK_ASSIGN, YjTaskAssign, qos=self.qos_reliable)
        self.topic_yj_resp = Topic(self.part_warning, TOPIC_YJ_TASK_RESPONSE, TaskResponse, qos=self.qos_reliable)

        # 单星域
        self.topic_command = Topic(self.part_sinsate, TOPIC_SINSATE_COMMAND, SinsateCommand, qos=self.qos_reliable)

        # 3. 创建发布者 (DataWriter)
        self.writer_zc_assign = DataWriter(self.part_recon, self.topic_zc_assign, qos=self.qos_reliable)
        self.writer_zc_resp = DataWriter(self.part_recon, self.topic_zc_resp, qos=self.qos_reliable)

        self.writer_yj_assign = DataWriter(self.part_warning, self.topic_yj_assign, qos=self.qos_reliable)
        self.writer_yj_resp = DataWriter(self.part_warning, self.topic_yj_resp, qos=self.qos_reliable)

        self.writer_command = DataWriter(self.part_sinsate, self.topic_command, qos=self.qos_reliable)

        # 4. 创建订阅者 (DataReader)
        self.reader_zc_req = DataReader(self.part_recon, self.topic_zc_req, qos=self.qos_reliable)
        self.reader_yj_guide = DataReader(self.part_warning, self.topic_yj_guide, qos=self.qos_reliable)

        logger.info("DDS 实体初始化完成，通道已就绪。")

    def start(self):
        """启动后台 DDS 需求监听线程"""
        if self.running:
            return
        self.running = True

        t_zc = threading.Thread(target=self._zc_listener_loop, name="DDS-ZC-Listener", daemon=True)
        t_yj = threading.Thread(target=self._yj_listener_loop, name="DDS-YJ-Listener", daemon=True)
        
        self.threads = [t_zc, t_yj]
        for t in self.threads:
            t.start()

        logger.info("============================================================")
        logger.info("  星间 DDS 任务规划节点已启动，持续监听星间数据总线...")
        logger.info(f"  * 监听主题: {TOPIC_ZC_REQ} (域 {self.domain_recon})")
        logger.info(f"  * 监听主题: {TOPIC_TARGET_GUIDE} (域 {self.domain_warning})")
        logger.info(f"  * 输出主题: {TOPIC_ZC_TASK_ASSIGN}, {TOPIC_YJ_TASK_ASSIGN}, {TOPIC_SINSATE_COMMAND}")
        logger.info("============================================================")

    def stop(self):
        """优雅停止节点"""
        logger.info("正在停止 DDS 节点管理器...")
        self.running = False
        for t in self.threads:
            t.join(timeout=1.0)
        logger.info("DDS 节点已安全停机。")

    # --------------------------------------------------------------------------
    # 3. 事件驱动监听循环
    # --------------------------------------------------------------------------
    def _zc_listener_loop(self):
        """侦察任务需求监听循环"""
        logger.info(f"开始监听侦察任务需求 [{TOPIC_ZC_REQ}]...")
        while self.running:
            try:
                for req in self.reader_zc_req.take_iter(timeout=duration(seconds=0.5)):
                    if not self.running:
                        break
                    logger.info(f"[收到侦察需求] ID: {req.zcRequestId}, 优先级: {req.priority}, 点目标数: {req.pointTargetCnt}, 面目标数: {req.areaTargetCnt}")
                    self._handle_zc_request(req)
            except Exception as e:
                if self.running:
                    logger.error(f"侦察监听处理异常: {e}", exc_info=True)

    def _yj_listener_loop(self):
        """预警目标引导监听循环"""
        logger.info(f"开始监听预警目标引导 [{TOPIC_TARGET_GUIDE}]...")
        while self.running:
            try:
                for guide in self.reader_yj_guide.take_iter(timeout=duration(seconds=0.5)):
                    if not self.running:
                        break
                    logger.info(f"[收到预警引导] ID: {guide.taskID}, 目标: {guide.viewID}, 轨迹点数: {guide.pointCnt}")
                    self._handle_yj_guide(guide)
            except Exception as e:
                if self.running:
                    logger.error(f"预警监听处理异常: {e}", exc_info=True)

    # --------------------------------------------------------------------------
    # 4. 业务处理与算法流水线调度
    # --------------------------------------------------------------------------
    def _handle_zc_request(self, req: ZcReq):
        """处理侦察需求：转为算法输入字典 -> 触发求解 -> 发布方案与动作指令"""
        t0 = time.time()
        payload = self.zc_req_to_payload(req)
        
        try:
            res = run(payload, self.work_dir)
            dur = round(time.time() - t0, 3)

            if isinstance(res, dict) and "metrics" in res:
                m = res["metrics"]
                logger.info(f"侦察规划求解成功！耗时: {dur}s, 目标完成率: {m.get('completionRate')*100:.1f}%, 评分: {m.get('overallScore')}")

                # 1. 发布任务分配数据 (ZC_TASK_ASSIGN_TOPIC)
                assign_list = res.get("taskAssignList", [])
                for assign_item in assign_list:
                    assign_msg = self.dict_to_zc_assign(assign_item)
                    self.writer_zc_assign.write(assign_msg)
                logger.info(f"  -> 已发布 {len(assign_list)} 份分配方案到 [{TOPIC_ZC_TASK_ASSIGN}]")

                # 2. 发布单星动作指令 (SINSATE_COMMAND_TOPIC)
                cmd_list = res.get("commandList", [])
                for cmd_item in cmd_list:
                    cmd_msg = self.dict_to_sinsate_command(cmd_item)
                    self.writer_command.write(cmd_msg)
                logger.info(f"  -> 已发布 {len(cmd_list)} 条单星控制指令到 [{TOPIC_SINSATE_COMMAND}]")

                # 3. 发布成功应答
                resp = TaskResponse(
                    sourceNode=1,
                    targetNode=req.sourceNode,
                    timestamp=format_ts_to_iso(time.time()),
                    taskID=req.zcRequestId,
                    status=0,
                    message=f"Planning Success: {len(cmd_list)} tasks scheduled in {dur}s"
                )
                self.writer_zc_resp.write(resp)

            else:
                logger.warning(f"侦察规划无可行解: {res}")
                resp = TaskResponse(
                    sourceNode=1, targetNode=req.sourceNode,
                    timestamp=format_ts_to_iso(time.time()),
                    taskID=req.zcRequestId, status=1, message="Planning Failed: No feasible schedule"
                )
                self.writer_zc_resp.write(resp)

        except Exception as e:
            logger.error(f"侦察任务规划调度异常: {e}", exc_info=True)
            resp = TaskResponse(
                sourceNode=1, targetNode=req.sourceNode,
                timestamp=format_ts_to_iso(time.time()),
                taskID=req.zcRequestId, status=2, message=f"Planning Error: {str(e)}"
            )
            self.writer_zc_resp.write(resp)

    def _handle_yj_guide(self, guide: TargetGuide):
        """处理预警引导：转为算法输入字典 -> 触发求解 -> 发布方案与动作指令"""
        t0 = time.time()
        payload = self.yj_guide_to_payload(guide)
        
        try:
            res = run(payload, self.work_dir)
            dur = round(time.time() - t0, 3)

            if isinstance(res, dict) and "metrics" in res:
                m = res["metrics"]
                logger.info(f"预警接力规划求解成功！耗时: {dur}s, 接力覆盖率: {m.get('completionRate')*100:.1f}%, 评分: {m.get('overallScore')}")

                # 1. 发布预警分配数据 (YJ_TASK_ASSIGN_TOPIC)
                assign_list = res.get("taskAssignList", [])
                for assign_item in assign_list:
                    assign_msg = self.dict_to_yj_assign(assign_item)
                    self.writer_yj_assign.write(assign_msg)
                logger.info(f"  -> 已发布 {len(assign_list)} 份预警分配方案到 [{TOPIC_YJ_TASK_ASSIGN}]")

                # 2. 发布单星动作指令 (SINSATE_COMMAND_TOPIC)
                cmd_list = res.get("commandList", [])
                for cmd_item in cmd_list:
                    cmd_msg = self.dict_to_sinsate_command(cmd_item)
                    self.writer_command.write(cmd_msg)
                logger.info(f"  -> 已发布 {len(cmd_list)} 条单星预警指令到 [{TOPIC_SINSATE_COMMAND}]")

                # 3. 发布成功应答
                resp = TaskResponse(
                    sourceNode=1,
                    targetNode=guide.sourceNode,
                    timestamp=format_ts_to_iso(time.time()),
                    taskID=guide.taskID,
                    status=0,
                    message=f"Warning Tracking Planning Success in {dur}s"
                )
                self.writer_yj_resp.write(resp)

            else:
                logger.warning(f"预警规划无可行解: {res}")
                resp = TaskResponse(
                    sourceNode=1, targetNode=guide.sourceNode,
                    timestamp=format_ts_to_iso(time.time()),
                    taskID=guide.taskID, status=1, message="Warning Planning Failed: No feasible schedule"
                )
                self.writer_yj_resp.write(resp)

        except Exception as e:
            logger.error(f"预警任务规划调度异常: {e}", exc_info=True)
            resp = TaskResponse(
                sourceNode=1, targetNode=guide.sourceNode,
                timestamp=format_ts_to_iso(time.time()),
                taskID=guide.taskID, status=2, message=f"Warning Planning Error: {str(e)}"
            )
            self.writer_yj_resp.write(resp)

    # --------------------------------------------------------------------------
    # 5. IDL 实体与算法 Payload 相互转换适配器
    # --------------------------------------------------------------------------
    @staticmethod
    def zc_req_to_payload(req: ZcReq) -> Dict[str, Any]:
        """将 CycloneDDS ZcReq 转换为算法核心所需的标准字典"""
        point_pos = [{"targetId": p.targetId, "lon": p.lon, "lat": p.lat, "alt": p.alt} for p in req.pointTargetPos]
        pt_type_names = ["Civilship", "Carrier", "Missile_launcher", "Unknown"]
        point_types = [pt_type_names[p.value] if p.value < len(pt_type_names) else "Unknown" for p in req.pointTargetType]
        
        payload_names = ["Panchromatic", "Infrared", "Multispectral", "Hyperspectral", "SAR", "Electronic", "Radar"]
        point_payloads = [payload_names[p.value] if p.value < len(payload_names) else "Panchromatic" for p in req.pointTargetPayload]

        area_pot1 = [{"areaId": a.areaId, "lon": a.lon, "lat": a.lat, "alt": a.alt} for a in req.areaPot1Pos]
        area_pot2 = [{"areaId": a.areaId, "lon": a.lon, "lat": a.lat, "alt": a.alt} for a in req.areaPot2Pos]
        area_pot3 = [{"areaId": a.areaId, "lon": a.lon, "lat": a.lat, "alt": a.alt} for a in req.areaPot3Pos]
        area_pot4 = [{"areaId": a.areaId, "lon": a.lon, "lat": a.lat, "alt": a.alt} for a in req.areaPot4Pos]

        area_type_names = ["Island", "Port", "City", "Unknown"]
        area_types = [area_type_names[a.value] if a.value < len(area_type_names) else "Unknown" for a in req.areaTargetType]
        area_payloads = [payload_names[p.value] if p.value < len(payload_names) else "Panchromatic" for p in req.areaTargetPayload]

        return {
            "dataname": {
                "validData": {
                    "sourceNode": req.sourceNode,
                    "targetNode": req.targetNode,
                    "timestamp": req.timestamp,
                    "zcRequestId": req.zcRequestId,
                    "priority": req.priority,
                    "taskStartTime": req.taskStartTime,
                    "taskEndTime": req.taskEndTime,
                    "pointTargetCnt": req.pointTargetCnt,
                    "pointTargetPos": point_pos,
                    "pointTargetType": point_types,
                    "pointTargetPriority": list(req.pointTargetPriority),
                    "pointTargetPayload": point_payloads,
                    "areaTargetCnt": req.areaTargetCnt,
                    "areaPot1Pos": area_pot1,
                    "areaPot2Pos": area_pot2,
                    "areaPot3Pos": area_pot3,
                    "areaPot4Pos": area_pot4,
                    "areaTargetType": area_types,
                    "areaTargetPriority": list(req.areaTargetPriority),
                    "areaTargetPayload": area_payloads
                }
            }
        }

    @staticmethod
    def yj_guide_to_payload(guide: TargetGuide) -> Dict[str, Any]:
        """将 CycloneDDS TargetGuide 转换为算法核心所需的标准字典"""
        points = [{"lon": p.lon, "lat": p.lat, "alt": p.alt} for p in guide.pointPosList]
        vels = [{"eastVel": v.east, "northVel": v.north, "upVel": v.up} for v in guide.pointVelList]
        task_type_str = "Tracking" if guide.taskType == YjTaskType.TRACKING else "Staring"

        return {
            "dataname": {
                "validData": {
                    "sourceNode": guide.sourceNode,
                    "targetNode": guide.targetNode,
                    "timestamp": guide.timestamp,
                    "taskID": guide.taskID,
                    "priority": guide.priority,
                    "taskType": task_type_str,
                    "taskStartTime": guide.taskStartTime,
                    "taskEndTime": guide.taskEndTime,
                    "viewID": guide.viewID,
                    "viewLon": guide.viewLon,
                    "viewLat": guide.viewLat,
                    "viewAlt": guide.viewAlt,
                    "pointCnt": guide.pointCnt,
                    "pointTime": list(guide.pointTime),
                    "pointPosList": points,
                    "pointVelList": vels
                }
            }
        }

    @staticmethod
    def dict_to_zc_assign(assign_dict: Dict[str, Any]) -> ZcTaskAssign:
        """将算法输出的任务分配字典序列化为 DDS ZcTaskAssign IDL 实体"""
        vd = assign_dict.get("dataname", {}).get("validData", assign_dict)
        pts = [
            PointTargetPosition(
                targetId=str(p.get("targetId", "")),
                lon=float(p.get("lon", 0.0)),
                lat=float(p.get("lat", 0.0)),
                alt=float(p.get("alt", 0.0))
            ) for p in vd.get("pointTargetPos", [])
        ]
        
        type_map = {"civilship": PointTargetType.POINT_CIVILSHIP, "carrier": PointTargetType.POINT_CARRIER, "missile_launcher": PointTargetType.POINT_MISSILE_LAUNCHER}
        pt_types = [type_map.get(str(t).lower(), PointTargetType.POINT_UNKNOWN) for t in vd.get("pointTargetType", [])]
        
        load_map = {"panchromatic": PayloadType.PAYLOAD_PANCHROMATIC, "infrared": PayloadType.PAYLOAD_INFRARED, "sar": PayloadType.PAYLOAD_SAR}
        pt_loads = [load_map.get(str(l).lower(), PayloadType.PAYLOAD_PANCHROMATIC) for l in vd.get("pointTargetPayload", [])]

        area1 = [AreaVertexPosition(areaId=str(a.get("areaId", "")), lon=float(a.get("lon", 0.0)), lat=float(a.get("lat", 0.0)), alt=float(a.get("alt", 0.0))) for a in vd.get("areaPot1Pos", [])]
        area2 = [AreaVertexPosition(areaId=str(a.get("areaId", "")), lon=float(a.get("lon", 0.0)), lat=float(a.get("lat", 0.0)), alt=float(a.get("alt", 0.0))) for a in vd.get("areaPot2Pos", [])]
        area3 = [AreaVertexPosition(areaId=str(a.get("areaId", "")), lon=float(a.get("lon", 0.0)), lat=float(a.get("lat", 0.0)), alt=float(a.get("alt", 0.0))) for a in vd.get("areaPot3Pos", [])]
        area4 = [AreaVertexPosition(areaId=str(a.get("areaId", "")), lon=float(a.get("lon", 0.0)), lat=float(a.get("lat", 0.0)), alt=float(a.get("alt", 0.0))) for a in vd.get("areaPot4Pos", [])]
        
        at_map = {"island": AreaTargetType.AREA_ISLAND, "port": AreaTargetType.AREA_PORT, "city": AreaTargetType.AREA_CITY}
        at_types = [at_map.get(str(t).lower(), AreaTargetType.AREA_UNKNOWN) for t in vd.get("areaTargetType", [])]
        at_loads = [load_map.get(str(l).lower(), PayloadType.PAYLOAD_PANCHROMATIC) for l in vd.get("areaTargetPayload", [])]

        return ZcTaskAssign(
            execNode=int(vd.get("execNode", 1)),
            missionID=int(vd.get("missionID", 1)),
            taskAssignTime=str(vd.get("taskAssignTime", "")),
            taskStartTime=str(vd.get("taskStartTime", "")),
            taskEndTime=str(vd.get("taskEndTime", "")),
            pointTargetCnt=len(pts),
            pointTargetPos=pts,
            pointTargetType=pt_types,
            pointTargetPriority=[int(p) for p in vd.get("pointTargetPriority", [])],
            pointTargetPayload=pt_loads,
            areaTargetCnt=len(area1),
            areaPot1Pos=area1,
            areaPot2Pos=area2,
            areaPot3Pos=area3,
            areaPot4Pos=area4,
            areaTargetType=at_types,
            areaTargetPriority=[int(p) for p in vd.get("areaTargetPriority", [])],
            areaTargetPayload=at_loads
        )

    @staticmethod
    def dict_to_yj_assign(assign_dict: Dict[str, Any]) -> YjTaskAssign:
        """将算法输出的预警任务分配字典序列化为 DDS YjTaskAssign IDL 实体"""
        vd = assign_dict.get("dataname", {}).get("validData", assign_dict)
        r_pos = vd.get("realPointPos", {})
        r_vel = vd.get("realPointVel", {})
        
        pred_pts = [GeoPosition(lon=float(p.get("lon", 0.0)), lat=float(p.get("lat", 0.0)), alt=float(p.get("alt", 0.0))) for p in vd.get("predPointPos", [])]
        pred_vels = [GeoVelocity(east=float(v.get("eastVel", 0.0)), north=float(v.get("northVel", 0.0)), up=float(v.get("upVel", 0.0))) for v in vd.get("predPointVel", [])]

        return YjTaskAssign(
            execNode=int(vd.get("execNode", 1)),
            missionID=int(vd.get("missionID", 1)),
            taskAssignTime=str(vd.get("taskAssignTime", "")),
            taskStartTime=str(vd.get("taskStartTime", "")),
            taskEndTime=str(vd.get("taskEndTime", "")),
            taskType=YjTaskType.TRACKING if str(vd.get("taskType", "Tracking")).lower() == "tracking" else YjTaskType.STARING,
            viewID=str(vd.get("viewID", "")),
            viewLon=float(vd.get("viewLon", 0.0)),
            viewLat=float(vd.get("viewLat", 0.0)),
            viewAlt=float(vd.get("viewAlt", 0.0)),
            realPointTime=str(vd.get("realPointTime", "")),
            realPointPos=GeoPosition(lon=float(r_pos.get("lon", 0.0)), lat=float(r_pos.get("lat", 0.0)), alt=float(r_pos.get("alt", 0.0))),
            realPointVel=GeoVelocity(east=float(r_vel.get("eastVel", 0.0)), north=float(r_vel.get("northVel", 0.0)), up=float(r_vel.get("upVel", 0.0))),
            predPointCnt=len(pred_pts),
            predPointTime=[str(t) for t in vd.get("predPointTime", [])],
            predPointPos=pred_pts,
            predPointVel=pred_vels
        )

    @staticmethod
    def dict_to_sinsate_command(cmd_dict: Dict[str, Any]) -> SinsateCommand:
        """将算法输出的单星动作指令字典序列化为 DDS SinsateCommand IDL 实体"""
        vd = cmd_dict.get("dataname", {}).get("validData", cmd_dict)
        return SinsateCommand(
            SCID=int(vd.get("SCID", 1)),
            imageSendTime=str(vd.get("imageSendTime", "")),
            rollAngle=float(vd.get("rollAngle", 0.0)),
            pitchAngle=float(vd.get("pitchAngle", 0.0)),
            yawAngle=float(vd.get("yawAngle", 0.0)),
            attiQuaterW=float(vd.get("attiQuaterW", 1.0)),
            attiQuaterX=float(vd.get("attiQuaterX", 0.0)),
            attiQuaterY=float(vd.get("attiQuaterY", 0.0)),
            attiQuaterZ=float(vd.get("attiQuaterZ", 0.0))
        )
