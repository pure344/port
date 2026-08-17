# 使用官方的 Python 瘦身版基础镜像，支持 ARM64/AMD64 多架构
FROM python:3.10-slim

# 设置工作目录
WORKDIR /app

# 更换 apt 源为阿里云（可选，加快国内构建速度，但在离线或非联网环境下无需关心，因为我们在你有网的本地先 build 好）
# 安装开源求解器 CoinOR-CBC
# 这在 Linux(AMD64/ARM64) 容器里会自动安装 `cbc` 命令到系统 PATH 中
RUN apt-get update && \
    apt-get install -y --no-install-recommends coinor-cbc && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# 将当前目录下的所有 Python 源码复制到容器内
# .dockerignore 会帮我们过滤掉 cbc.exe / test / __pycache__ 等不需要的文件
COPY . /app/

# 设置环境变量
ENV APP_PORT=8080
ENV WORK_DIR=/app
ENV RUN_MODE=server

# 暴露规划服务端口
EXPOSE 8080

# 启动 HTTP 规划服务
CMD ["python", "-u", "main.py"]
