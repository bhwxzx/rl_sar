#!/usr/bin/env python3
"""Passive Linux FDILink attitude preflight; never starts the controller."""

import argparse
import binascii
from collections import Counter
from contextlib import contextmanager
import fcntl
import math
import os
from pathlib import Path
import select
import signal
import stat
import struct
import subprocess
import sys
import termios
import time


def load_limits(path):
    # Use the deployed configuration explicitly, never a guessed workspace copy.
    import yaml

    with open(path, encoding="utf-8") as source:
        document = yaml.safe_load(source)
    if not isinstance(document, dict) or not isinstance(document.get("LW"), dict):
        raise ValueError("配置必须包含 LW 映射")
    limits = []
    for key in ("policy_entry_angle_deg", "policy_entry_stable_time", "trusted_imu_timeout"):
        value = document["LW"].get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{key} 必须是数值")
        value = float(value)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{key} 必须有限且大于零")
        limits.append(value)
    if limits[0] >= 90:
        raise ValueError("policy_entry_angle_deg 必须小于 90 度")
    return tuple(limits)


def crc8(data):
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8C if value & 1 else 0)
    return value


class FrameParser:
    """FDILink framing from fdilink_frame_parser.h, including ignored types."""

    LENGTHS = {0x40: 56, 0x41: 48, 0x42: 72, 0x5C: 32}

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        while self.buffer:
            if self.buffer[0] != 0xFC:
                position = self.buffer.find(b"\xfc")
                del self.buffer[:position if position >= 0 else len(self.buffer)]
                yield None, "帧同步丢失"
                continue
            if len(self.buffer) < 7:
                return
            kind, length = self.buffer[1:3]
            if (kind not in (*self.LENGTHS, 0xF0, 0x50)
                    or (kind in self.LENGTHS and length != self.LENGTHS[kind])
                    or crc8(self.buffer[:4]) != self.buffer[4]):
                del self.buffer[0]
                yield None, "帧头/长度/CRC8 错误"
                continue
            size = length + 8
            if len(self.buffer) < size:
                return
            payload = bytes(self.buffer[7:size - 1])
            checksum = int.from_bytes(self.buffer[5:7], "big")
            if self.buffer[size - 1] != 0xFD or binascii.crc_hqx(payload, 0) != checksum:
                del self.buffer[0]
                yield None, "帧尾/CRC16 错误"
                continue
            del self.buffer[:size]
            yield kind, payload


def decode_attitude(payload):
    values = struct.unpack("<10fq", payload)
    if not all(math.isfinite(value) for value in values[:10]):
        raise ValueError("AHRS 包含非有限数值")
    w, x, y, z = values[6:10]
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    if not 0.9 <= norm <= 1.1:
        raise ValueError("四元数模长超出 [0.9, 1.1]")
    # Driver device_type=1: qz(pi)*qy(pi) * q * qx(pi), then normalize
    # as the real subscriber does. Sensor-provided Euler angles are not used.
    w, x, y, z = w/norm, x/norm, -y/norm, -z/norm
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return tuple(math.degrees(value) for value in (roll, pitch, yaw)), values[10]


class AttitudeCheck:
    def __init__(self, angle, stable_time, timeout):
        self.angle, self.stable_time, self.timeout = angle, stable_time, timeout
        self.last_time = self.last_stamp = self.stable_since = None
        self.latest = None
        self.minimum = [math.inf] * 3
        self.maximum = [-math.inf] * 3
        self.valid_count = 0
        self.faults = Counter()
        self.last_fault = "尚无有效 AHRS 数据"
        self.data_ok = False

    def reject(self, reason):
        self.faults[reason] += 1
        self.last_fault = reason
        self.stable_since = None
        self.data_ok = False

    def accept(self, payload, now):
        try:
            angles, stamp = decode_attitude(payload)
        except ValueError as error:
            self.reject(str(error))
            return
        if stamp <= 0 or (self.last_stamp is not None and stamp <= self.last_stamp):
            self.reject("AHRS 时间戳未递增/发生回退")
            # Keep the high-water mark: a sensor restart requires a new check.
            return
        self.last_stamp = stamp
        if self.last_time is not None and now - self.last_time > self.timeout:
            self.reject("AHRS 接收间隔超限")
        self.last_time = now
        self.latest = angles
        self.data_ok = True
        self.valid_count += 1
        self.minimum = [min(a, b) for a, b in zip(self.minimum, angles)]
        self.maximum = [max(a, b) for a, b in zip(self.maximum, angles)]
        if abs(angles[0]) <= self.angle and abs(angles[1]) <= self.angle:
            if self.stable_since is None:
                self.stable_since = now
        else:
            self.stable_since = None

    def stable_duration(self):
        if self.stable_since is None or self.last_time is None:
            return 0.0
        # Only newly received samples advance the interval, never wall-clock wait.
        return self.last_time - self.stable_since

    def result(self, now):
        if self.last_time is None:
            return 2, self.last_fault
        if now - self.last_time > self.timeout:
            return 2, "检查结束时 AHRS 数据已过期"
        if not self.data_ok:
            return 2, self.last_fault
        if abs(self.latest[0]) > self.angle or abs(self.latest[1]) > self.angle:
            return 1, "检查结束时 roll 或 pitch 超出角度阈值"
        if self.stable_duration() < self.stable_time:
            return 1, "检查结束时连续合格时长不足"
        return 0, "结束时姿态角度及连续采样条件合格"


def check_port_free(port, own_pid=None):
    # fuser checks pre-existing readers; TIOCEXCL alone cannot exclude them.
    result = subprocess.run(["fuser", str(port)], capture_output=True, text=True, timeout=3)
    if result.returncode not in (0, 1):
        raise RuntimeError(f"无法检查串口占用: {result.stderr.strip()}")
    if result.returncode == 1 and result.stderr.strip():
        raise RuntimeError(f"串口占用检查不完整: {result.stderr.strip()}")
    try:
        holders = {int(value) for value in result.stdout.split()}
    except ValueError as error:
        raise RuntimeError("无法解析串口占用检查结果") from error
    holders.discard(own_pid)
    if holders:
        raise RuntimeError(f"IMU 串口正在被进程 {sorted(holders)} 占用；请先正常停止驱动")


@contextmanager
def passive_serial(port):
    path = Path(port).resolve(strict=True)
    if not stat.S_ISCHR(path.stat().st_mode):
        raise ValueError("IMU 端口必须是字符设备")
    for motor in ("/dev/ttyLegRight", "/dev/ttyLegLeft"):
        if Path(motor).exists() and path == Path(motor).resolve():
            raise ValueError("拒绝访问电机板串口")
    check_port_free(path)
    fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    saved = None
    exclusive = False
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        exclusive = True
        check_port_free(path, os.getpid())
        saved = termios.tcgetattr(fd)
        settings = termios.tcgetattr(fd)
        settings[0] = settings[1] = settings[3] = 0
        settings[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        settings[4] = settings[5] = termios.B921600
        settings[6][termios.VMIN] = settings[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        yield fd
    finally:
        # Preserve cleanup even when restoration itself fails. Such a failure
        # propagates to main and prevents reporting PASS.
        try:
            if saved is not None:
                termios.tcsetattr(fd, termios.TCSANOW, saved)
                if termios.tcgetattr(fd) != saved:
                    raise RuntimeError("串口参数恢复校验失败")
        finally:
            try:
                if exclusive:
                    fcntl.ioctl(fd, termios.TIOCNXCL)
            finally:
                os.close(fd)


def sample(fd, check, duration):
    # Drain startup bytes for 0.5 s, then discard any remaining host-side queue.
    warmup_end = time.monotonic() + 0.5
    while time.monotonic() < warmup_end:
        if select.select([fd], [], [], max(0, warmup_end - time.monotonic()))[0]:
            if not os.read(fd, 4096):
                raise RuntimeError("IMU 串口断开")
    termios.tcflush(fd, termios.TCIFLUSH)
    parser = FrameParser()
    end = time.monotonic() + duration
    while True:
        remaining = end - time.monotonic()
        if remaining <= 0:
            break
        if not select.select([fd], [], [], min(remaining, check.timeout))[0]:
            continue
        data = os.read(fd, 4096)
        now = time.monotonic()
        if not data:
            raise RuntimeError("IMU 串口断开")
        for kind, payload in parser.feed(data):
            if kind is None:
                check.reject(payload)
            elif kind == 0x41:
                check.accept(payload, now)
    return check.result(time.monotonic())


def interrupt(signum, frame):
    raise KeyboardInterrupt


def main(argv=None):
    parser = argparse.ArgumentParser(description="启动前被动检查 LW 机身姿态（Linux，FDILink device_type=1）")
    parser.add_argument("--config", required=True, type=Path, help="将要启动的部署包 policy/LW/base.yaml")
    parser.add_argument("--port", default="/dev/fdilink_ahrs", help="仅限 IMU 串口，默认 /dev/fdilink_ahrs")
    parser.add_argument("--duration", type=float, default=5.0, help="有效采样时长（秒），默认 5，另有 0.5 秒预热")
    args = parser.parse_args(argv)
    previous = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        angle, stable, timeout = load_limits(args.config)
        if not math.isfinite(args.duration) or args.duration <= stable:
            raise ValueError("duration 必须有限且大于配置中的连续合格时长")
        check = AttitudeCheck(angle, stable, timeout)
        print(f"配置: {args.config.resolve()}\nIMU: {args.port}; 921600 baud; device_type=1", flush=True)
        print(f"要求: |roll|、|pitch| <= {angle:g}°，结束前连续 {stable:g}s；"
              f"接收间隔/末帧时效 <= {timeout:g}s；yaw 仅显示", flush=True)
        with passive_serial(args.port) as fd:
            code, reason = sample(fd, check, args.duration)
        # Report only after host serial parameters have been restored.
        print(f"有效 AHRS: {check.valid_count}; 结束时连续合格: {check.stable_duration():.3f}s")
        if check.latest is not None:
            print("角度(度)       最新         最小         最大")
            for index, name in enumerate(("roll", "pitch", "yaw")):
                print(f"{name:5s} {check.latest[index]:12.3f} {check.minimum[index]:12.3f} {check.maximum[index]:12.3f}")
        if check.faults:
            print("采集异常计数（重新满足连续条件后可恢复）: " + str(dict(check.faults)))
        print(f"{'PASS' if code == 0 else 'FAIL'}: {reason}")
        print("仅证明本次 IMU 输出符合姿态预检条件；不证明安装/零偏正确，也不保证闭环稳定。不会自动启动程序。")
        return code
    except KeyboardInterrupt:
        print("检查已中断，未通过。", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"ERROR: {error}；未通过。", file=sys.stderr)
        return 2
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    sys.exit(main())
