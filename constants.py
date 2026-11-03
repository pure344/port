# ==============================================================================
# 文件名称: constants.py
# 功能描述: 全局常量配置与 DDS 协议工具模块
#          统一管理业务阈值、默认参数、时间转换、四元数计算与 DDS 报文封装
# ==============================================================================

import math
import time
from datetime import datetime, timezone
from typing import Tuple, Dict, Any

# ------------------------------------------------------------------------------
# 1. 业务时长与时间窗相关常量
# ------------------------------------------------------------------------------
# 观测任务的默认固定时长 (秒)
OBSERVATION_DEFAULT_DURATION = 10

# 极端场景下盲注兜底逻辑的提前量 (秒) - 默认提前 1 小时
EMERGENCY_BLIND_TTC_ADVANCE = 3600

# 极端场景下盲注兜底逻辑的窗口时长 (秒) - 默认 10 分钟
EMERGENCY_BLIND_TTC_DURATION = 600

# Unix 时间戳有效性校验阈值
UNIX_EPOCH_THRESHOLD = 1000000000

# 常规测控方案的默认兜底时长 (秒)
DEFAULT_TTC_DURATION = 60

# 数传结束到数据产品分发开始的默认延时时长 (秒) - 默认 30 分钟 (1800 秒)
DEFAULT_DISTRIBUTE_DELAY = 1800

# ------------------------------------------------------------------------------
# 2. 卫星性能与业务默认参数
# ------------------------------------------------------------------------------
# 默认的星上处理速率乘数
DEFAULT_PROCESS_RATE = 0.5

# 默认的数传下行速率乘数
DEFAULT_TRANSMISSION_RATE = 2.0

# 预警任务高优放大倍数 (确保优先打断常规任务并完成接力)
WARNING_PRIORITY_MULTIPLIER = 100000.0

# ------------------------------------------------------------------------------
# 3. 算法与数学模型相关常量
# ------------------------------------------------------------------------------
# 混合整数线性规划 (MILP) 中的大 M 法常数 (7 天秒数)
BIG_M = 604800.0

# 浮点数解转换为二值变量 (0/1) 时的截断阈值
BINARIZATION_THRESHOLD = 0.5

# ------------------------------------------------------------------------------
# 4. DDS 领域与主题定义
# ------------------------------------------------------------------------------
DDS_DOMAINS = {
    "SINSATE": {"name": "单星数据域", "id": 1},
    "NETWORK": {"name": "网络域", "id": 2},
    "GLOBAL": {"name": "全局数据域", "id": 3},
    "RECON": {"name": "侦察任务域", "id": 4},
    "WARNING": {"name": "预警任务域", "id": 5}
}

TOPIC_ZC_REQ = "ZC_REQ_TOPIC"
TOPIC_ZC_TASK_ASSIGN = "ZC_TASK_ASSIGN_TOPIC"
TOPIC_TARGET_GUIDE = "TARGET_GUIDE_TOPIC"
TOPIC_YJ_TASK_ASSIGN = "YJ_TASK_ASSIGN_TOPIC"
TOPIC_SINSATE_COMMAND = "SINSATE_COMMAND_TOPIC"

# ------------------------------------------------------------------------------
# 5. 时间与空间动力学工具函数
# ------------------------------------------------------------------------------
def parse_time_to_ts(t_val: Any) -> float:
    """
    通用时间解析函数：
    支持 ISO 8601 UTC ('2024-01-01T00:00:00.000Z', '2024-01-01T00:00:00Z')、
    标准空格时间 ('2026-07-28 08:00:00') 以及原始数值时间戳。
    统一返回 UNIX 浮点秒。
    """
    if isinstance(t_val, (int, float)):
        return float(t_val)
    if not t_val or not isinstance(t_val, str):
        return 0.0

    s = t_val.strip()
    if s.endswith('Z') or s.endswith('z'):
        s = s[:-1] + '+00:00'

    try:
        dt = datetime.fromisoformat(s)
        if dt.tzinfo is not None:
            return dt.timestamp()
        else:
            return dt.replace(tzinfo=timezone.utc).timestamp()
    except Exception:
        for fmt in ("%Y-%m-%d %H:%M:%S", "%Y-%m-%dT%H:%M:%S", "%Y-%m-%d"):
            try:
                return datetime.strptime(s, fmt).replace(tzinfo=timezone.utc).timestamp()
            except ValueError:
                pass

    raise ValueError(f"无法解析时间字符串: {t_val}")

def format_ts_to_iso(ts: float) -> str:
    """
    将 Unix 浮点时间戳格式化为 DDS 规范要求的 ISO 8601 UTC 字符串：
    'YYYY-MM-DDTHH:MM:SS.000Z'
    """
    if ts <= 0:
        return "1970-01-01T00:00:00.000Z"
    dt = datetime.fromtimestamp(ts, tz=timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S.000Z")

def euler_to_quaternion(roll_deg: float, pitch_deg: float = 0.0, yaw_deg: float = 0.0) -> Tuple[float, float, float, float]:
    """
    欧拉角（度）转换为标准归一化姿态角四元数 (w, x, y, z)。
    采用航天标准 Z-Y-X (yaw-pitch-roll) 内旋顺序。
    """
    phi = math.radians(roll_deg) / 2.0
    theta = math.radians(pitch_deg) / 2.0
    psi = math.radians(yaw_deg) / 2.0

    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)
    sin_theta = math.sin(theta)
    cos_theta = math.cos(theta)
    sin_psi = math.sin(psi)
    cos_psi = math.cos(psi)

    w = cos_phi * cos_theta * cos_psi + sin_phi * sin_theta * sin_psi
    x = sin_phi * cos_theta * cos_psi - cos_phi * sin_theta * sin_psi
    y = cos_phi * sin_theta * cos_psi + sin_phi * cos_theta * sin_psi
    z = cos_phi * cos_theta * sin_psi - sin_phi * sin_theta * cos_psi

    # 归一化保障数值稳定性
    norm = math.sqrt(w*w + x*x + y*y + z*z) or 1.0
    return (
        round(w / norm, 6),
        round(x / norm, 6),
        round(y / norm, 6),
        round(z / norm, 6)
    )

def make_dds_wrapper(topic: str, domain_key: str, summary: str, valid_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    将规划结果封装为标准 DDS 规范字典 (包含 dataname, validData 与 metadata)。
    """
    domain_info = DDS_DOMAINS.get(domain_key, {"name": "任务域", "id": 0})
    return {
        "dataname": {
            "validData": valid_data,
            "metadata": {
                "coreSet": {
                    "manageInfo": {
                        "summary": summary,
                        "format": "JSON",
                        "encoding": "UTF-8",
                        "version": "2026.10.16"
                    },
                    "DDS": {
                        "domain": domain_info["name"],
                        "topic": topic,
                        "QoS": "可靠传输",
                        "frequency": "事件"
                    },
                    "timeInfo": {
                        "createTime": format_ts_to_iso(time.time()),
                        "timeFormat": "YYYY-MM-DDTHH:MM:SS.000Z"
                    }
                }
            }
        }
    }
