#!/usr/bin/python3

import argparse
import glob
import logging
import os
import shutil
import signal
import subprocess
import tarfile
import time

from concurrent.futures import ThreadPoolExecutor

# =========================================================
# 全局退出标志
# =========================================================

running = True

# =========================================================
# 默认配置
# =========================================================

DEFAULT_DURATION = 60
DEFAULT_OVERLAP = 3
DEFAULT_MAX_GB = 50
DEFAULT_COMPRESS_WORKERS = 2

DEFAULT_TOPICS = [
    '/tf',
]

# =========================================================
# 日志
# =========================================================

BASE_DIR = '/home/kane/siasun/siasun_test'

LOG_DIR = os.path.join(
    BASE_DIR,
    'siasun_amr_bag_log'
)

os.makedirs(LOG_DIR, exist_ok=True)

# 单个日志最大大小
MAX_LOG_SIZE = 10 * 1024 * 1024

# 最多保留日志数
MAX_LOG_COUNT = 10

logger = logging.getLogger('siasun_recorder')

logger.setLevel(logging.INFO)

logger.propagate = False

current_handler = None

# =========================================================
# 创建日志 handler
# =========================================================

def create_log_handler():

    timestamp = time.strftime('%Y%m%d_%H%M%S')

    log_file = os.path.join(
        LOG_DIR,
        f'siasun_{timestamp}.log'
    )

    handler = logging.FileHandler(log_file)

    formatter = logging.Formatter(
        '[%(asctime)s] [%(levelname)s] %(message)s'
    )

    handler.setFormatter(formatter)

    return handler

# =========================================================
# 初始化 logger
# =========================================================

current_handler = create_log_handler()

logger.addHandler(current_handler)

# =========================================================
# 清理旧日志
# =========================================================

def cleanup_old_logs():

    logs = sorted(
        glob.glob(
            os.path.join(LOG_DIR, 'siasun_*.log')
        )
    )

    while len(logs) > MAX_LOG_COUNT:

        oldest = logs.pop(0)

        try:

            os.remove(oldest)

        except Exception:

            pass

# =========================================================
# 日志轮转
# =========================================================

def rotate_log_if_needed():

    global current_handler

    try:

        if current_handler.stream.tell() < MAX_LOG_SIZE:

            return

    except Exception:

        return

    logger.removeHandler(current_handler)

    current_handler.close()

    current_handler = create_log_handler()

    logger.addHandler(current_handler)

    cleanup_old_logs()

# =========================================================
# 日志封装
# =========================================================

def log_info(msg):

    logger.info(msg)

    rotate_log_if_needed()

def log_warn(msg):

    logger.warning(msg)

    rotate_log_if_needed()

def log_error(msg):

    logger.error(msg)

    rotate_log_if_needed()

# =========================================================
# signal
# =========================================================

def signal_handler(signum, frame):

    global running

    log_warn(f'[退出] 收到信号: {signum}')

    running = False

signal.signal(signal.SIGINT, signal_handler)

signal.signal(signal.SIGTERM, signal_handler)

# =========================================================
# 工具函数
# =========================================================

def now_str():

    return time.strftime('%Y%m%d_%H%M%S')

def interruptible_sleep(seconds):

    global running

    end_time = time.monotonic() + seconds

    while running and time.monotonic() < end_time:

        remain = end_time - time.monotonic()

        time.sleep(min(0.5, remain))

def get_dir_size_gb(path):

    total = 0

    for dirpath, _, filenames in os.walk(path):

        for f in filenames:

            fp = os.path.join(dirpath, f)

            try:

                total += os.path.getsize(fp)

            except OSError:

                pass

    return total / (1024 ** 3)

# =========================================================
# 清理旧 bag
# =========================================================

def cleanup_old_bags(save_dir, max_gb):

    archives = sorted(
        glob.glob(os.path.join(save_dir, 'siasun_*'))
    )

    while archives and get_dir_size_gb(save_dir) > max_gb:

        oldest = archives.pop(0)

        try:

            if os.path.isdir(oldest):

                shutil.rmtree(oldest)

            else:

                os.remove(oldest)

            log_warn(f'[清理] 删除旧 bag: {oldest}')

        except Exception as e:

            log_error(f'[清理] 删除失败: {e}')

            break

# =========================================================
# 压缩 bag
# =========================================================

def compress_bag(bag_path):

    tar_path = bag_path + '.tar.gz'

    try:

        with tarfile.open(tar_path, 'w:gz') as tar:

            tar.add(
                bag_path,
                arcname=os.path.basename(bag_path)
            )

        shutil.rmtree(bag_path)

        log_info(f'[压缩] 完成: {tar_path}')

    except Exception as e:

        log_error(f'[压缩] 失败: {e}')

# =========================================================
# 启动 rosbag
# =========================================================

def launch_rosbag(save_dir, topics):

    bag_name = f'siasun_{now_str()}'

    bag_path = os.path.join(save_dir, bag_name)

    if topics is None:

        cmd = [
            'ros2',
            'bag',
            'record',
            '-a',
            '--compression-mode',
            'file',
            '--compression-format',
            'zstd',
            '-o',
            bag_path,
        ]

    else:

        cmd = [
            'ros2',
            'bag',
            'record',
            *topics,
            '--compression-mode',
            'file',
            '--compression-format',
            'zstd',
            '-o',
            bag_path,
        ]

    log_info(f'[录制] 启动 rosbag: {bag_name}')

    process = subprocess.Popen(
        cmd,

        start_new_session=True,

        stdout=subprocess.DEVNULL,

        stderr=subprocess.STDOUT,
    )

    log_info(f'[录制] pid={process.pid}')

    return process, bag_path, bag_name

# =========================================================
# 停止 rosbag
# =========================================================

def stop_rosbag(process, bag_name):

    if process is None:
        return

    if process.poll() is not None:
        return

    log_info(f'[停止] {bag_name}')

    try:

        os.killpg(
            os.getpgid(process.pid),
            signal.SIGINT
        )

        process.wait(timeout=15)

    except subprocess.TimeoutExpired:

        log_warn('[警告] rosbag 未退出，强制 kill')

        try:

            os.killpg(
                os.getpgid(process.pid),
                signal.SIGKILL
            )

        except ProcessLookupError:

            pass

    except ProcessLookupError:

        pass

    log_info(f'[录制] 结束: {bag_name}')

# =========================================================
# 后处理
# =========================================================

def post_process(bag_path, save_dir, max_gb):

    global running

    if not running:

        return

    if os.path.exists(bag_path):

        compress_bag(bag_path)

    cleanup_old_bags(save_dir, max_gb)

# =========================================================
# 主循环
# =========================================================

def run(
    save_dir,
    topics,
    duration,
    overlap,
    max_gb,
    compress_workers,
):

    global running

    if overlap >= duration:

        raise ValueError('overlap 必须小于 duration')

    executor = ThreadPoolExecutor(
        max_workers=compress_workers
    )

    cleanup_old_bags(save_dir, max_gb)

    process, bag_path, bag_name = launch_rosbag(
        save_dir,
        topics,
    )

    segment_start = time.monotonic()

    try:

        while running:

            # -------------------------------------------------
            # rosbag 崩溃恢复
            # -------------------------------------------------

            if process.poll() is not None and running:

                log_error('[错误] rosbag 异常退出，自动重启')

                process, bag_path, bag_name = launch_rosbag(
                    save_dir,
                    topics,
                )

                segment_start = time.monotonic()

            # -------------------------------------------------
            # overlap
            # -------------------------------------------------

            elapsed = time.monotonic() - segment_start

            wait_time = duration - overlap - elapsed

            if wait_time > 0:

                interruptible_sleep(wait_time)

            if not running:

                break

            next_process, next_bag_path, next_bag_name = launch_rosbag(
                save_dir,
                topics,
            )

            next_start = time.monotonic()

            log_info(f'[切换] overlap {overlap}s')

            interruptible_sleep(overlap)

            stop_rosbag(process, bag_name)

            if running:

                executor.submit(
                    post_process,
                    bag_path,
                    save_dir,
                    max_gb,
                )

            process = next_process

            bag_path = next_bag_path

            bag_name = next_bag_name

            segment_start = next_start

    finally:

        log_warn('[退出] 正在停止 recorder')

        stop_rosbag(process, bag_name)

        executor.shutdown(wait=False)

        log_warn('[退出] recorder 已退出')

# =========================================================
# main
# =========================================================

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument('save_dir')

    parser.add_argument('--all', action='store_true')

    parser.add_argument('--topics', nargs='+')

    parser.add_argument(
        '--duration',
        type=int,
        default=DEFAULT_DURATION,
    )

    parser.add_argument(
        '--overlap',
        type=int,
        default=DEFAULT_OVERLAP,
    )

    parser.add_argument(
        '--max-gb',
        type=float,
        default=DEFAULT_MAX_GB,
    )

    parser.add_argument(
        '--compress-workers',
        type=int,
        default=DEFAULT_COMPRESS_WORKERS,
    )

    args = parser.parse_args()

    if args.all:

        topics = None

    elif args.topics:

        topics = args.topics

    else:

        topics = DEFAULT_TOPICS

    os.makedirs(args.save_dir, exist_ok=True)

    log_info('================================================')

    log_info('[启动] siasun recorder')

    log_info('================================================')

    log_info(f'[启动] save_dir={args.save_dir}')

    log_info(f'[启动] duration={args.duration}')

    log_info(f'[启动] overlap={args.overlap}')

    log_info(f'[启动] max_gb={args.max_gb}')

    run(
        save_dir=args.save_dir,
        topics=topics,
        duration=args.duration,
        overlap=args.overlap,
        max_gb=args.max_gb,
        compress_workers=args.compress_workers,
    )

if __name__ == '__main__':

    main()