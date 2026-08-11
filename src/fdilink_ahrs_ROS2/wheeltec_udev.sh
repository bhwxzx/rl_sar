#!/usr/bin/env bash

set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
    echo "请使用 sudo 运行此脚本以安装 IMU udev 规则。" >&2
    exit 1
fi

rules_file=/etc/udev/rules.d/99-wheeltec-fdilink-ahrs.rules
umask 022
{
    # CP2102，串口序列号 0003。
    printf '%s\n' 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0003", MODE:="0777", SYMLINK+="fdilink_ahrs"'
    # CH9102，系统安装了 CH343 驱动，串口序列号 0003。
    printf '%s\n' 'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0003", MODE:="0777", SYMLINK+="fdilink_ahrs"'
    # CH9102，设备由通用 ACM 驱动识别，串口序列号 0003。
    printf '%s\n' 'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0003", MODE:="0777", SYMLINK+="fdilink_ahrs"'
    # CH340 通常没有唯一序列号；运行前必须确认主机上只有目标设备匹配。
    printf '%s\n' 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="fdilink_ahrs"'
} > "${rules_file}"
chmod 0644 "${rules_file}"

udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
udevadm settle

echo "已安装 ${rules_file}；请重新插拔 IMU 后检查 /dev/fdilink_ahrs。"
