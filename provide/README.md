# 卫星任务规划服务 - 星间 DDS 通信节点部署指南

本系统基于 MILP (混合整数线性规划) 模型与 CBC 求解器，提供卫星集群观测、处理、传输任务的智能规划服务。系统完全基于 OMG DDS（Eclipse CycloneDDS）标准实现，作为**纯 DDS 星间数据总线节点**常驻运行，支持在 ARM64 架构及 Linux 环境下部署。

---

## 目录结构说明

```text
.
├── docker-compose.yml           # 推荐：一键编排启动文件（含 host 网络与 tmpfs 内存优化）
├── README.md                    # 本说明文档
└── sat-planning_v1.0.0_arm64.tar # 离线容器镜像包 (ARM64 架构)
```

---

## 部署与启动说明 (离线环境)

### 1. 导入离线镜像

将压缩包上传至目标星载计算机或测试服务器，执行导入命令：

```bash
docker load -i sat-planning_v1.0.0_arm64.tar
```

### 2. 一键启动服务

推荐使用 `docker-compose` 启动（配置中已开启 `network_mode: host` 以接入宿主机的星间 DDS 组播网卡）：

```bash
# 在包含 docker-compose.yml 的目录下执行
docker-compose up -d
```

*(若使用原生 Docker 命令启动：`docker run -d --name sat-planning --net=host --tmpfs /tmp:rw,size=128m --restart=always sat-planning:v1.0.0`)*

---

## DDS 通信主题与接口说明

规划节点启动后自动加入星间 DDS 数据网络，监听需求主题并在规划完成后自动发布分配与指令：

| 通信方向 | 数据域 | 域 ID | 主题名 (Topic) | IDL 结构 | QoS | 频次 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **订阅 (Sub)** | 侦察任务域 | 4 | `ZC_REQ_TOPIC` | `TX::zc_req::ZcReq` | Reliable (可靠传输) | 事件驱动 |
| **订阅 (Sub)** | 预警任务域 | 5 | `TARGET_GUIDE_TOPIC` | `TX::target_guide::TargetGuide` | Reliable (可靠传输) | 事件驱动 |
| **发布 (Pub)** | 侦察任务域 | 4 | `ZC_TASK_ASSIGN_TOPIC` | `TX::zc_task_assign::ZcTaskAssign` | Reliable (可靠传输) | 事件驱动 |
| **发布 (Pub)** | 预警任务域 | 5 | `YJ_TASK_ASSIGN_TOPIC` | `TX::yj_task_assign::YjTaskAssign` | Reliable (可靠传输) | 事件驱动 |
| **发布 (Pub)** | 单星数据域 | 1 | `SINSATE_COMMAND_TOPIC` | `TX::sinsate_command::SinsateCommand` | Reliable (可靠传输) | 事件驱动 |

---

## 测试与联调

可直接运行自动化测试脚本，模拟地面站与卫星节点在 DDS 总线上进行发布与订阅闭环测试：

```bash
python test/test_dds.py
```
