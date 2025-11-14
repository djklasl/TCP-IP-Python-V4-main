# -*- coding: utf-8 -*-
# 吸盘/夹爪 Modbus RTU 示例（直连RS-485，最简版，仅使用 dobot_api.py 的接口）

import re
import sys
import time
from dobot_api import DobotApiDashboard


def main():
    """固定参数版：参数在代码内配置，不从终端传入。"""
    # 配置参数（请按现场设备修改）
    ip = "111.111.130.198"   # 机器人控制器 IP
    slave_id = 9              # 夹爪 Modbus 从站ID
    baud = 115200             # 串口波特率
    parity = 'N'              # 'N' 无校验, 'E' 偶校验, 'O' 奇校验
    stopbit = 1               # 停止位 1 或 2

    dash = DobotApiDashboard(ip, 29999)

    # 可选：确保末端为485并上电、设置串口
    try:
        print(dash.SetToolMode(1, 0))     # 1=485模式
        print(dash.SetToolPower(1))       # 工具口上电
        print(dash.SetTool485(baud, parity, stopbit))
        time.sleep(0.05)
    except Exception as e:
        print('SetTool* 调用异常(可忽略):', e)

    # 建立 Modbus RTU 主站
    resp = dash.ModbusRTUCreate(slave_id=slave_id, baud=baud, parity=parity, data_bit=8, stop_bit=stopbit)
    print('ModbusRTUCreate:', resp)

    # 激活吸盘
    dash.SetHoldRegs(0, 1000, 1, "1", 'U16')
    time.sleep(0.05)

    #设置自动模式并开始调节（rACT=1 + rGTO=1 => value 9）
    dash.SetHoldRegs(0, 1000, 1, "9", 'U16')

    #开始吸取
    dash.SetHoldRegs(0, 1001, 1, "50", 'U16')

    time.sleep(0.1)
    print('ModbusClose:', dash.ModbusClose(0))


if __name__ == '__main__':
    main()
