#!/bin/bash

set -e

# =========================================================
# ROS 环境
# =========================================================

source /opt/ros/humble/setup.bash

# source ~/amr_ws/install/setup.bash

# =========================================================
# 配置
# =========================================================

BASE_DIR="/home/kane/siasun/siasun_test"

BAG_DIR="${BASE_DIR}/siasun_amr_bag"

PID_DIR="${BASE_DIR}/run"

PID_FILE="${PID_DIR}/siasun_recorder.pid"

RECORDER="${BASE_DIR}/shell/siasun_recorder.py"

MAX_GB=3

# =========================================================
# 创建目录
# =========================================================

mkdir -p "${BAG_DIR}"

mkdir -p "${PID_DIR}"

# =========================================================
# 防重复启动
# =========================================================

if [ -f "${PID_FILE}" ]; then

    OLD_PID=$(cat "${PID_FILE}")

    if ps -p "${OLD_PID}" > /dev/null 2>&1; then

        echo "[错误] recorder 已运行"
        echo "[错误] pid=${OLD_PID}"

        exit 1

    else

        echo "[警告] 删除失效 pid 文件"

        rm -f "${PID_FILE}"
    fi
fi

# =========================================================
# 启动
# =========================================================

echo "================================================"

echo "[启动] siasun recorder"

echo "================================================"

nohup setsid python3 -u "${RECORDER}" \
    "${BAG_DIR}" \
    --max-gb "${MAX_GB}" \
    --all \
    >/dev/null 2>&1 &

PID=$!

echo "${PID}" > "${PID_FILE}"

echo "[启动] pid=${PID}"

echo "[启动] 完成"

