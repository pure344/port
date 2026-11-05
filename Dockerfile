# 使用官方的 Python 瘦身版基础镜像，支持 ARM64/AMD64 多架构
FROM python:3.10-slim

# 设置工作目录
WORKDIR /app

# 安装开源求解器 CoinOR-CBC
RUN apt-get update && \
    apt-get install -y --no-install-recommends coinor-cbc && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# 安装 OMG DDS 核心中间件库 CycloneDDS
RUN pip install --no-cache-dir cyclonedds

# 将当前目录下的所有 Python 源码复制到容器内
COPY . /app/

# 设置环境变量
ENV WORK_DIR=/app

# 启动星间 DDS 任务规划守护节点
CMD ["python", "-u", "main.py"]
