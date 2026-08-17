#!/bin/sh
# ============================================================
# 卫星任务规划系统 - 容器启动脚本
# ============================================================

# 加载配置文件中的环境变量
if [ -f /app/config/app.conf ]; then
    set -a
    source /app/config/app.conf
    set +a
fi

# 设置默认环境变量
export APP_PORT=${APP_PORT:-8080}
export LOG_LEVEL=${LOG_LEVEL:-INFO}

# 打印启动信息
echo "========================================"
echo "Starting Satellite Planning Service"
echo "Port: ${APP_PORT}"
echo "Log Level: ${LOG_LEVEL}"
echo "========================================"

# 启动 Python HTTP 服务
exec python /app/main.py
