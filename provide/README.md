# 卫星任务规划服务 - 项目说明与部署指南

本系统基于 MILP (混合整数线性规划) 模型与 CBC 求解器，提供卫星集群观测、处理、传输任务的智能规划服务。系统已完成容器化适配，支持在 ARM64 架构及 Linux 环境下部署运行。

---

## 目录结构说明

```text
.
├── docker-compose.yml           # 推荐：一键编排与启动配置文件（含 tmpfs 内存盘优化）
├── README.md                    # 本说明文档
└── sat-planning_v1.0.0_arm64.tar # 离线容器镜像包 (ARM64 架构)
```

*(注：以上为交付部署包内容，源码及开发配置文件已在容器镜像中固化)*

---

## 部署与启动说明 (离线环境)

系统已完全固化在离线镜像包中，无需联网即可在甲方服务器完成部署。

### 1. 导入离线镜像

将压缩包上传至目标服务器（需已安装 Docker 环境），执行导入命令：

```bash
docker load -i sat-planning_v1.0.0_arm64.tar
```

### 2. 一键启动服务

推荐使用 `docker-compose` 启动，配置文件中已包含将中间产物写入内存 (`tmpfs`) 的极速优化机制：

```bash
# 在包含 docker-compose.yml 的目录下执行
docker-compose up -d
```

*(如果目标机器未安装 docker-compose，也可使用原生命令启动：`docker run -d --name sat-planning -p 8080:8080 --tmpfs /tmp --restart=always sat-planning:v1.0.0`)*

---

## 接口使用说明

服务启动后监听 `8080` 端口。

### 1. 健康检查接口 (GET)

* **URL**: `http://localhost:8080/health/live`
* **Method**: `GET`
* **Response**:
  ```json
  {
    "status": "UP"
  }
  ```

### 2. 任务规划接口 (POST)

* **URL**: `http://localhost:8080/`
* **Method**: `POST`
* **Content-Type**: `application/json`
* **Body**: 包含需求单、候选任务列表、资源及路由信息的完整 JSON 数据。
* **Response**: 包含观测方案、处理方案、传输方案及评测指标的完整结果 JSON。

---

## 测试命令示例

```bash
# 验证健康检查
curl http://localhost:8080/health/live

# 发送任务规划请求
curl -X POST http://localhost:8080 \
  -H "Content-Type: application/json" \
  -d @test.json
```
