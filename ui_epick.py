# -*- coding: utf-8 -*-
"""Tkinter 图形界面模块：提供机械臂连接、点动、运动与反馈显示等 UI 功能。
+ 集成 Robotiq EPick 吸盘控制功能 (内嵌修复版)
"""

from threading import Thread
import time
from tkinter import *
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText
from dobot_api import *
import json
import threading
import math
import re
from files.alarmController import alarm_controller_list
from files.alarmServo import alarm_servo_list

# --- 1. 内嵌 Robotiq 依赖逻辑 (解决导入报错与编码问题) ---
ROBOTIQ_EPICK_DEFAULT_BAUDRATE = 115200
ROBORIQ_EPICK_WRITE_ADDRESS = 0x03E8
ERROR_CODE = -1
SUCCESS_CODE = 0

def binaryStringToDecimal(binaryStr):
    return int(binaryStr, 2)

def decimalToBinaryString(num, length=8):
    return bin(num)[2:].zfill(length)

# 命令构建器
CMD_HANDLER = {
    "ACTION": lambda x: binaryStringToDecimal("".join(x) + "00000000"),
    "MAX_VACUUM": lambda x: binaryStringToDecimal("00000000" + decimalToBinaryString(x)),
    "MIN_VACUUM": lambda x: decimalToBinaryString(x),
    "TIMEOUT": lambda x: decimalToBinaryString(x)
}

ACTION_CMD = {
    "ACT": {"CLEAR_FAULT_STATUS": "0", "ENABLE": "1"},
    "MOD": {"AUTO": "00", "ADVANCED": "01"},
    "GTO": {"HOLD": "0", "CONTROLED": "1"},
    "ATR": {"NORMAL": "0", "RELEASE_WITH_NO_TIMEOUT": "1"}
}

class RobotiqEpick:
    """
    Robotiq EPick 控制类 (修复版)
    特点：复用主程序的 dashboard 连接，不再单独创建 Socket，防止冲突和报错。
    """
    RELEASE_VACUUM = 255

    def __init__(self, dashboard_client):
        # 直接接收已连接的 dashboard 对象
        self.dashboard = dashboard_client
        self.ModbusIndex = None
        self.write_lock = threading.Lock()
        
        # 尝试初始化 485 (忽略错误，因为可能已经初始化过)
        try:
            self.dashboard.SetToolMode(1,1,1)
            self.dashboard.SetTool485(115200, "N", 1)
        except: pass

    def create_modbus_channel(self) -> tuple[int, int]:
        # 创建 Modbus 连接
        response = self.dashboard.ModbusCreate("127.0.0.1" ,60000,9,1)
        error_id = 0
        modbus_index = -1
        # 解析返回字符串 "0,1" -> error_id=0, index=1
        if isinstance(response, str):
            numbers = [int(item) for item in re.findall(r"-?\d+", response)]
            if numbers: error_id = numbers[0]
            if len(numbers) > 1:
                modbus_index = numbers[1]
                self.ModbusIndex = modbus_index
        return error_id, modbus_index

    def close_modbus_channel(self) -> str:
        if self.ModbusIndex is None: return "Modbus Not Connected"
        result = self.dashboard.ModbusClose(self.ModbusIndex)
        self.ModbusIndex = None
        return result

    def write_by_485(self, address: int, datas: list[int]) -> str:
        if self.ModbusIndex is None: return "Error: No Modbus Index"
        with self.write_lock:
            # 每次写入前确保 485 参数正确
            self.dashboard.SetTool485(115200, "N", 1)
            time.sleep(0.01)
            # 构造数据字符串 "{123,456}"
            value_str = ",".join(str(int(value)) for value in datas)
            val_tab = f"{{{value_str}}}"
            # 发送指令
            result = self.dashboard.SetHoldRegs(self.ModbusIndex, address, len(datas), val_tab)
            time.sleep(0.5) # 给硬件反应时间
            return result

    def init(self):
        # 1. 清除故障
        cmd = CMD_HANDLER["ACTION"]([
            ACTION_CMD["ATR"]["NORMAL"], ACTION_CMD["GTO"]["HOLD"],
            ACTION_CMD["MOD"]["ADVANCED"], ACTION_CMD["ACT"]["CLEAR_FAULT_STATUS"],
        ])
        _maxVacuum = CMD_HANDLER["MAX_VACUUM"](self.RELEASE_VACUUM)
        self.write_by_485(ROBORIQ_EPICK_WRITE_ADDRESS, [cmd, _maxVacuum])
        
        # 2. 激活设备
        resetCMD = CMD_HANDLER["ACTION"]([
            ACTION_CMD["ATR"]["NORMAL"], ACTION_CMD["GTO"]["HOLD"],
            ACTION_CMD["MOD"]["ADVANCED"], ACTION_CMD["ACT"]["ENABLE"],
        ])
        return self.write_by_485(ROBORIQ_EPICK_WRITE_ADDRESS, [resetCMD, _maxVacuum])

    def grip(self, max_v: int, min_v: int, timeout: int):
        # 1. 重置状态
        resetCMD = CMD_HANDLER["ACTION"]([
            ACTION_CMD["ATR"]["NORMAL"], ACTION_CMD["GTO"]["HOLD"],
            ACTION_CMD["MOD"]["ADVANCED"], ACTION_CMD["ACT"]["CLEAR_FAULT_STATUS"],
        ])
        self.write_by_485(ROBORIQ_EPICK_WRITE_ADDRESS, [resetCMD])
        
        # 2. 设置参数 (最大真空、最小真空、超时)
        _maxVacuum = CMD_HANDLER["MAX_VACUUM"](100 - max_v)
        timeoutAndMin = binaryStringToDecimal(
            CMD_HANDLER["TIMEOUT"](math.ceil(timeout / 100)) + CMD_HANDLER["MIN_VACUUM"](100 - min_v)
        )
        # 写入 Action 和 Params
        cmd_enable = CMD_HANDLER["ACTION"]([
                ACTION_CMD["ATR"]["NORMAL"], ACTION_CMD["GTO"]["HOLD"],
                ACTION_CMD["MOD"]["ADVANCED"], ACTION_CMD["ACT"]["ENABLE"],
            ])
        self.write_by_485(ROBORIQ_EPICK_WRITE_ADDRESS, [cmd_enable])
        self.write_by_485(ROBORIQ_EPICK_WRITE_ADDRESS + 1, [_maxVacuum, timeoutAndMin])

        # 3. 执行吸取
        gripCMD = CMD_HANDLER["ACTION"]([
            ACTION_CMD["ATR"]["NORMAL"], ACTION_CMD["GTO"]["CONTROLED"],
            ACTION_CMD["MOD"]["ADVANCED"], ACTION_CMD["ACT"]["ENABLE"],
        ])
        return self.write_by_485(ROBORIQ_EPICK_WRITE_ADDRESS, [gripCMD])

    def release(self):
        # 1. 重置
        resetCMD = CMD_HANDLER["ACTION"]([
            ACTION_CMD["ATR"]["NORMAL"], ACTION_CMD["GTO"]["HOLD"],
            ACTION_CMD["MOD"]["ADVANCED"], ACTION_CMD["ACT"]["CLEAR_FAULT_STATUS"],
        ])
        self.write_by_485(ROBORIQ_EPICK_WRITE_ADDRESS, [resetCMD])
        
        # 2. 执行释放
        cmd = CMD_HANDLER["ACTION"]([
            ACTION_CMD["ATR"]["NORMAL"], ACTION_CMD["GTO"]["CONTROLED"],
            ACTION_CMD["MOD"]["ADVANCED"], ACTION_CMD["ACT"]["ENABLE"],
        ])
        _maxVacuum = CMD_HANDLER["MAX_VACUUM"](self.RELEASE_VACUUM)
        self.write_by_485(ROBORIQ_EPICK_WRITE_ADDRESS, [cmd, _maxVacuum])
        time.sleep(1)
        
        # 3. 回到保持状态
        holdCMD = CMD_HANDLER["ACTION"]([
            ACTION_CMD["ATR"]["NORMAL"], ACTION_CMD["GTO"]["HOLD"],
            ACTION_CMD["MOD"]["ADVANCED"], ACTION_CMD["ACT"]["ENABLE"],
        ])
        return self.write_by_485(ROBORIQ_EPICK_WRITE_ADDRESS, [holdCMD])

# -----------------------------------------------------------------------------
# 常量定义 (保持原 ui.py 不变)
# -----------------------------------------------------------------------------
LABEL_JOINT = [["J1-", "J2-", "J3-", "J4-", "J5-", "J6-"],
               ["J1:", "J2:", "J3:", "J4:", "J5:", "J6:"],
               ["J1+", "J2+", "J3+", "J4+", "J5+", "J6+"]]

LABEL_COORD = [["X-", "Y-", "Z-", "Rx-", "Ry-", "Rz-"],
               ["X:", "Y:", "Z:", "Rx:", "Ry:", "Rz:"],
               ["X+", "Y+", "Z+", "Rx+", "Ry+", "Rz+"]]

LABEL_ROBOT_MODE = {
    1: "ROBOT_MODE_INIT", 2: "ROBOT_MODE_BRAKE_OPEN", 3: "",
    4: "ROBOT_MODE_DISABLED", 5: "ROBOT_MODE_ENABLE", 6: "ROBOT_MODE_DRAGE",
    7: "ROBOT_MODE_RUNNING", 8: "ROBOT_MODE_RECORDING", 9: "ROBOT_MODE_ERROR",
    10: "ROBOT_MODE_PAUSE", 11: "ROBOT_MODE_JOG"
}

class RobotUI(object):
    """机器人 UI 主类"""

    def __init__(self):
        self.root = Tk()
        self.root.title("Python demo V4 (With Gripper)")
        # 增加窗口高度以容纳吸盘模块：原 850 -> 960
        self.root.geometry("900x960")

        self.global_state = {}
        self.button_list = []
        self.entry_dict = {}
        
        # --- 新增：吸盘控制器引用 ---
        self.epick = None
        # --------------------------

        # 1. Robot Connect
        self.frame_robot = LabelFrame(self.root, text="Robot Connect",
                                      labelanchor="nw", bg="#FFFFFF", width=870, height=120, border=2)
        self.label_ip = Label(self.frame_robot, text="IP Address:")
        self.label_ip.place(rely=0.2, x=10)
        ip_port = StringVar(self.root, value="111.111.130.198")
        self.entry_ip = Entry(self.frame_robot, width=12, textvariable=ip_port)
        self.entry_ip.place(rely=0.2, x=90)
        self.label_dash = Label(self.frame_robot, text="Dashboard Port:")
        self.label_dash.place(rely=0.2, x=210)
        dash_port = IntVar(self.root, value=29999)
        self.entry_dash = Entry(self.frame_robot, width=7, textvariable=dash_port)
        self.entry_dash.place(rely=0.2, x=320)
        self.label_feed = Label(self.frame_robot, text="Feedback Port:")
        self.label_feed.place(rely=0.2, x=420)
        feed_port = IntVar(self.root, value=30004)
        self.entry_feed = Entry(self.frame_robot, width=7, textvariable=feed_port)
        self.entry_feed.place(rely=0.2, x=520)
        self.button_connect = self.set_button(master=self.frame_robot,
                                              text="Connect", rely=0.6, x=630, command=self.connect_port)
        self.button_connect["width"] = 10
        self.global_state["connect"] = False

        # 2. Dashboard Function
        self.frame_dashboard = LabelFrame(self.root, text="Dashboard Function",
                                          labelanchor="nw", bg="#FFFFFF", pady=10, width=870, height=120, border=2)
        self.button_enable = self.set_button(master=self.frame_dashboard,
                                             text="Enable", rely=0.1, x=10, command=self.enable)
        self.button_enable["width"] = 7
        self.button_start_drag = self.set_button(master=self.frame_dashboard,
                                                 text="StartDrag", rely=0.1, x=100, command=self.start_drag)
        self.button_start_drag["width"] = 10
        self.global_state["enable"] = False
        self.global_state["drag"] = False

        # Drag Sensitivity
        Label(self.frame_dashboard, text="Drag Sensitivity:").place(rely=0.55, x=430)
        Label(self.frame_dashboard, text="Axis:").place(rely=0.55, x=550)
        self.spin_drag_axis = Spinbox(self.frame_dashboard, from_=0, to=6, width=4)
        self.spin_drag_axis.place(rely=0.55, x=590)
        Label(self.frame_dashboard, text="Value:").place(rely=0.55, x=640)
        self.spin_drag_value = Spinbox(self.frame_dashboard, from_=1, to=90, width=5)
        self.spin_drag_value.place(rely=0.55, x=690)
        self.set_button(master=self.frame_dashboard,
                        text="Apply", rely=0.5, x=750, command=self.set_drag_sensitivity)

        self.set_button(master=self.frame_dashboard,
                        text="ClearError", rely=0.1, x=200, command=self.clear_error)
        self.label_speed = Label(self.frame_dashboard, text="Speed Ratio:")
        self.label_speed.place(rely=0.1, x=430)
        s_value = StringVar(self.root, value="50")
        self.entry_speed = Entry(self.frame_dashboard, width=6, textvariable=s_value)
        self.entry_speed.place(rely=0.1, x=520)
        self.label_cent = Label(self.frame_dashboard, text="%")
        self.label_cent.place(rely=0.1, x=550)
        self.set_button(master=self.frame_dashboard,
                        text="Confirm", rely=0.1, x=586, command=self.confirm_speed)
        
        # Digital IO
        self.label_digitial = Label(self.frame_dashboard, text="Digital Outputs: Index:")
        self.label_digitial.place(rely=0.55, x=10)
        i_value = IntVar(self.root, value="1")
        self.entry_index = Entry(self.frame_dashboard, width=5, textvariable=i_value)
        self.entry_index.place(rely=0.55, x=160)
        self.label_status = Label(self.frame_dashboard, text="Status:")
        self.label_status.place(rely=0.55, x=220)
        self.combo_status = ttk.Combobox(self.frame_dashboard, width=5)
        self.combo_status["value"] = ("On", "Off")
        self.combo_status.current(0)
        self.combo_status["state"] = "readonly"
        self.combo_status.place(rely=0.55, x=275)
        self.set_button(self.frame_dashboard, "Confirm", rely=0.55, x=350, command=self.confirm_do)

        # 3. Move Function
        self.frame_move = LabelFrame(self.root, text="Move Function", labelanchor="nw",
                                     bg="#FFFFFF", width=870, pady=10, height=130, border=2)
        self.set_move(text="X:", label_value=10, default_value="600", entry_value=40, rely=0.1, master=self.frame_move)
        self.set_move(text="Y:", label_value=110, default_value="-260", entry_value=140, rely=0.1, master=self.frame_move)
        self.set_move(text="Z:", label_value=210, default_value="380", entry_value=240, rely=0.1, master=self.frame_move)
        self.set_move(text="Rx:", label_value=310, default_value="170", entry_value=340, rely=0.1, master=self.frame_move)
        self.set_move(text="Ry:", label_value=410, default_value="12", entry_value=440, rely=0.1, master=self.frame_move)
        self.set_move(text="Rz:", label_value=510, default_value="140", entry_value=540, rely=0.1, master=self.frame_move)
        self.set_button(master=self.frame_move, text="MovJ", rely=0.05, x=610, command=self.movj)
        self.set_button(master=self.frame_move, text="MovL", rely=0.05, x=700, command=self.movl)
        
        self.set_move(text="J1:", label_value=10, default_value="0", entry_value=40, rely=0.5, master=self.frame_move)
        self.set_move(text="J2:", label_value=110, default_value="-20", entry_value=140, rely=0.5, master=self.frame_move)
        self.set_move(text="J3:", label_value=210, default_value="-80", entry_value=240, rely=0.5, master=self.frame_move)
        self.set_move(text="J4:", label_value=310, default_value="30", entry_value=340, rely=0.5, master=self.frame_move)
        self.set_move(text="J5:", label_value=410, default_value="90", entry_value=440, rely=0.5, master=self.frame_move)
        self.set_move(text="J6:", label_value=510, default_value="120", entry_value=540, rely=0.5, master=self.frame_move)
        self.set_button(master=self.frame_move, text="MovJ", rely=0.45, x=610, command=self.joint_movj)

        # -------------------------------------------------------------------------
        # [新增] 4. Gripper Control (保持 ui.py 布局风格)
        # -------------------------------------------------------------------------
        self.frame_gripper = LabelFrame(self.root, text="Gripper Control (Robotiq EPick)", labelanchor="nw",
                                        bg="#FFFFFF", width=870, pady=10, height=110, border=2, fg="blue")
        
        # ID & Mode
        Label(self.frame_gripper, text="ID:", bg="white").place(rely=0.2, x=10)
        self.epick_id = Entry(self.frame_gripper, width=5)
        self.epick_id.insert(0, "9")
        self.epick_id.place(rely=0.2, x=35)

        Label(self.frame_gripper, text="Mode:", bg="white").place(rely=0.2, x=80)
        self.epick_mode = Entry(self.frame_gripper, width=5)
        self.epick_mode.insert(0, "1")
        self.epick_mode.place(rely=0.2, x=125)

        # Params
        Label(self.frame_gripper, text="Max(%):", bg="white").place(rely=0.2, x=170)
        self.epick_max = Entry(self.frame_gripper, width=5)
        self.epick_max.insert(0, "80")
        self.epick_max.place(rely=0.2, x=225)

        Label(self.frame_gripper, text="Min(%):", bg="white").place(rely=0.2, x=270)
        self.epick_min = Entry(self.frame_gripper, width=5)
        self.epick_min.insert(0, "20")
        self.epick_min.place(rely=0.2, x=325)

        Label(self.frame_gripper, text="Timeout:", bg="white").place(rely=0.2, x=370)
        self.epick_timeout = Entry(self.frame_gripper, width=6)
        self.epick_timeout.insert(0, "0")
        self.epick_timeout.place(rely=0.2, x=430)

        # Buttons (初始化、吸取、释放)
        self.btn_epick_init = self.set_button(self.frame_gripper, "Init Gripper", rely=0.55, x=10, command=self.epick_init)
        self.btn_epick_init["width"] = 12
        
        self.btn_epick_grip = self.set_button(self.frame_gripper, "Grip", rely=0.55, x=140, command=self.epick_grip)
        self.btn_epick_grip["width"] = 10
        self.btn_epick_grip["bg"] = "#e1ffea" # 浅绿色高亮
        
        self.btn_epick_release = self.set_button(self.frame_gripper, "Release", rely=0.55, x=250, command=self.epick_release)
        self.btn_epick_release["width"] = 10
        self.btn_epick_release["bg"] = "#ffe1e1" # 浅红色高亮
        # -------------------------------------------------------------------------

        # 5. Feedback / Log
        self.frame_feed_log = Frame(self.root, bg="#FFFFFF", width=870, pady=10, height=400, border=2)
        self.frame_feed = LabelFrame(self.frame_feed_log, text="Feedback", labelanchor="nw",
                                     bg="#FFFFFF", width=550, height=150)
        self.frame_feed.place(relx=0, rely=0, relheight=1)
        self.set_label(self.frame_feed, text="Current Speed Ratio:", rely=0.05, x=10)
        self.label_feed_speed = self.set_label(self.frame_feed, "", rely=0.05, x=145)
        self.set_label(self.frame_feed, text="%", rely=0.05, x=175)
        self.set_label(self.frame_feed, text="Robot Mode:", rely=0.1, x=10)
        self.label_robot_mode = self.set_label(self.frame_feed, "", rely=0.1, x=95)
        self.label_feed_dict = {}
        self.set_feed(LABEL_JOINT, 9, 52, 74, 117)
        self.set_feed(LABEL_COORD, 165, 209, 231, 272)
        self.set_label(self.frame_feed, "Digital Inputs:", rely=0.8, x=11)
        self.label_di_input = self.set_label(self.frame_feed, "", rely=0.8, x=100)
        self.set_label(self.frame_feed, "Digital Outputs:", rely=0.85, x=10)
        self.label_di_output = self.set_label(self.frame_feed, "", rely=0.85, x=100)
        self.frame_err = LabelFrame(self.frame_feed, text="Error Info", labelanchor="nw",
                                    bg="#FFFFFF", width=180, height=50)
        self.frame_err.place(relx=0.65, rely=0, relheight=0.7)
        self.text_err = ScrolledText(self.frame_err, width=170, height=50, relief="flat")
        self.text_err.place(rely=0, relx=0, relheight=0.7, relwidth=1)
        self.set_button(self.frame_feed, "Clear", rely=0.71, x=487, command=self.clear_error_info)
        self.frame_log = LabelFrame(self.frame_feed_log, text="Log", labelanchor="nw",
                                    bg="#FFFFFF", width=300, height=150)
        self.frame_log.place(relx=0.65, rely=0, relheight=1)
        self.text_log = ScrolledText(self.frame_log, width=270, height=140, relief="flat")
        self.text_log.place(rely=0, relx=0, relheight=1, relwidth=1)
        self.client_dash = None
        self.client_feed = None
        self.alarm_controller_dict = self.convert_dict(alarm_controller_list)
        self.alarm_servo_dict = self.convert_dict(alarm_servo_list)

    def convert_dict(self, alarm_list):
        alarm_dict = {}
        for i in alarm_list:
            alarm_dict[i["id"]] = i
        return alarm_dict

    def mainloop(self):
        self.root.mainloop()

    def pack(self):
        self.frame_robot.pack()
        self.frame_dashboard.pack()
        self.frame_move.pack()
        # 插入新增加的吸盘面板
        self.frame_gripper.pack()
        self.frame_feed_log.pack()

    def set_move(self, text, label_value, default_value, entry_value, rely, master):
        self.label = Label(master, text=text)
        self.label.place(rely=rely, x=label_value)
        value = StringVar(self.root, value=default_value)
        self.entry_temp = Entry(master, width=6, textvariable=value)
        self.entry_temp.place(rely=rely, x=entry_value)
        self.entry_dict[text] = self.entry_temp

    def move_jog(self, text):
        if self.global_state["connect"]:
            if text[0] == "J":
                self.client_dash.MoveJog(text)
            else:
                self.client_dash.MoveJog(text,coordtype=1,user=0,tool=0)

    def move_stop(self, event):
        if self.global_state["connect"]:
            self.client_dash.MoveJog("")

    def set_button(self, master, text, rely, x, **kargs):
        self.button = Button(master, text=text, padx=5, command=kargs["command"])
        self.button.place(rely=rely, x=x)
        if text != "Connect":
            self.button["state"] = "disable"
            self.button_list.append(self.button)
        return self.button

    def set_button_bind(self, master, text, rely, x, **kargs):
        self.button = Button(master, text=text, padx=5)
        self.button.bind("<ButtonPress-1>", lambda event: self.move_jog(text=text))
        self.button.bind("<ButtonRelease-1>", self.move_stop)
        self.button.place(rely=rely, x=x)
        if text != "Connect":
            self.button["state"] = "disable"
            self.button_list.append(self.button)
        return self.button

    def set_label(self, master, text, rely, x):
        self.label = Label(master, text=text)
        self.label.place(rely=rely, x=x)
        return self.label

    # --- 吸盘功能回调 ---
    def epick_init(self):
        if not self.epick: return
        try:
            self.epick.init()
        except Exception as e:
            messagebox.showerror("EPick Error", str(e))

    def epick_grip(self):
        if not self.epick: return
        try:
            mx = int(self.epick_max.get())
            mn = int(self.epick_min.get())
            tm = int(self.epick_timeout.get())
            self.epick.grip(mx, mn, tm)
            print(f"EPick Grip (Max={mx}, Min={mn})")
        except Exception as e:
            messagebox.showerror("EPick Error", str(e))

    def epick_release(self):
        if not self.epick: return
        try:
            self.epick.release()
            print("EPick Release")
        except Exception as e:
            messagebox.showerror("EPick Error", str(e))
    # ------------------

    def connect_port(self):
        if self.global_state["connect"]:
            # 断开
            print("断开成功")
            
            # 关闭吸盘连接
            if self.epick:
                self.epick.close_modbus_channel()
                self.epick = None
            
            self.client_dash.close()
            self.client_feed.close()
            self.client_dash = None
            self.client_feed = None
            for i in self.button_list:
                i["state"] = "disable"
            self.button_connect["text"] = "Connect"
            self.global_state["drag"] = False
            try:
                self.button_start_drag["relief"] = "raised"
                self.button_start_drag["text"] = "StartDrag"
            except Exception: pass
        else:
            # 连接
            try:
                print("连接成功")
                self.client_dash = DobotApiDashboard(
                    self.entry_ip.get(), int(self.entry_dash.get()), self.text_log)
                self.client_feed = DobotApiFeedBack(
                    self.entry_ip.get(), int(self.entry_feed.get()), self.text_log)
                
                # --- 连接成功后初始化吸盘 (复用连接) ---
                self.epick = RobotiqEpick(self.client_dash)
                # 尝试创建Modbus信道(如已创建则忽略)
                e_id, m_idx = self.epick.create_modbus_channel()
                print(f"EPick Modbus: Err={e_id}, Idx={m_idx}")
                # -----------------------------------

            except Exception as e:
                messagebox.showerror("Attention!", f"Connection Error:{e}")
                return
            for i in self.button_list:
                i["state"] = "normal"
            self.button_connect["text"] = "Disconnect"
        self.global_state["connect"] = not self.global_state["connect"]
        self.set_feed_back()

    def set_feed_back(self):
        if self.global_state["connect"]:
            thread = Thread(target=self.feed_back)
            thread.setDaemon(True)
            thread.start()

    def enable(self):
        if self.global_state["enable"]:
            self.client_dash.DisableRobot()
            self.button_enable["text"] = "Enable"
        else:
            self.client_dash.EnableRobot()
            self.button_enable["text"] = "Disable"
        self.global_state["enable"] = not self.global_state["enable"]

    def start_drag(self):
        if not self.global_state.get("connect"): return
        if not self.global_state.get("drag", False):
            try: self.client_dash.StartDrag()
            except Exception: return
            self.global_state["drag"] = True
            self.button_start_drag["relief"] = "sunken"
            self.button_start_drag["text"] = "StopDrag"
        else:
            try: self.client_dash.StopDrag()
            except Exception: return
            self.global_state["drag"] = False
            self.button_start_drag["relief"] = "raised"
            self.button_start_drag["text"] = "StartDrag"

    def set_drag_sensitivity(self):
        try:
            axis = int(self.spin_drag_axis.get())
            value = int(self.spin_drag_value.get())
            self.client_dash.DragSensivity(axis, value)
        except Exception: pass

    def clear_error(self):
        self.client_dash.ClearError()

    def confirm_speed(self):
        self.client_dash.SpeedFactor(int(self.entry_speed.get()))

    def movj(self):
        self.client_dash.MovJ(float(self.entry_dict["X:"].get()), float(self.entry_dict["Y:"].get()), float(self.entry_dict["Z:"].get()),
                              float(self.entry_dict["Rx:"].get()), float(self.entry_dict["Ry:"].get()), float(self.entry_dict["Rz:"].get()),0)

    def movl(self):
        self.client_dash.MovL(float(self.entry_dict["X:"].get()), float(self.entry_dict["Y:"].get()), float(self.entry_dict["Z:"].get()),
                              float(self.entry_dict["Rx:"].get()), float(self.entry_dict["Ry:"].get()), float(self.entry_dict["Rz:"].get()),0)

    def joint_movj(self):
        self.client_dash.MovJ(float(self.entry_dict["J1:"].get()), float(self.entry_dict["J2:"].get()), float(self.entry_dict["J3:"].get()),
                                   float(self.entry_dict["J4:"].get()), float(self.entry_dict["J5:"].get()), float(self.entry_dict["J6:"].get()),1)

    def confirm_do(self):
        if self.combo_status.get() == "On":
            self.client_dash.DO(int(self.entry_index.get()), 1)
        else:
            self.client_dash.DO(int(self.entry_index.get()), 0)

    def set_feed(self, text_list, x1, x2, x3, x4):
        self.set_button_bind(self.frame_feed, text_list[0][0], rely=0.2, x=x1, command=lambda: self.move_jog(text_list[0][0]))
        self.set_button_bind(self.frame_feed, text_list[0][1], rely=0.3, x=x1, command=lambda: self.move_jog(text_list[0][1]))
        self.set_button_bind(self.frame_feed, text_list[0][2], rely=0.4, x=x1, command=lambda: self.move_jog(text_list[0][2]))
        self.set_button_bind(self.frame_feed, text_list[0][3], rely=0.5, x=x1, command=lambda: self.move_jog(text_list[0][3]))
        self.set_button_bind(self.frame_feed, text_list[0][4], rely=0.6, x=x1, command=lambda: self.move_jog(text_list[0][4]))
        self.set_button_bind(self.frame_feed, text_list[0][5], rely=0.7, x=x1, command=lambda: self.move_jog(text_list[0][5]))

        self.set_label(self.frame_feed, text_list[1][0], rely=0.21, x=x2)
        self.set_label(self.frame_feed, text_list[1][1], rely=0.31, x=x2)
        self.set_label(self.frame_feed, text_list[1][2], rely=0.41, x=x2)
        self.set_label(self.frame_feed, text_list[1][3], rely=0.51, x=x2)
        self.set_label(self.frame_feed, text_list[1][4], rely=0.61, x=x2)
        self.set_label(self.frame_feed, text_list[1][5], rely=0.71, x=x2)

        self.label_feed_dict[text_list[1][0]] = self.set_label(self.frame_feed, " ", rely=0.21, x=x3)
        self.label_feed_dict[text_list[1][1]] = self.set_label(self.frame_feed, " ", rely=0.31, x=x3)
        self.label_feed_dict[text_list[1][2]] = self.set_label(self.frame_feed, " ", rely=0.41, x=x3)
        self.label_feed_dict[text_list[1][3]] = self.set_label(self.frame_feed, " ", rely=0.51, x=x3)
        self.label_feed_dict[text_list[1][4]] = self.set_label(self.frame_feed, " ", rely=0.61, x=x3)
        self.label_feed_dict[text_list[1][5]] = self.set_label(self.frame_feed, " ", rely=0.71, x=x3)

        self.set_button_bind(self.frame_feed, text_list[2][0], rely=0.2, x=x4, command=lambda: self.move_jog(text_list[2][0]))
        self.set_button_bind(self.frame_feed, text_list[2][1], rely=0.3, x=x4, command=lambda: self.move_jog(text_list[2][1]))
        self.set_button_bind(self.frame_feed, text_list[2][2], rely=0.4, x=x4, command=lambda: self.move_jog(text_list[2][2]))
        self.set_button_bind(self.frame_feed, text_list[2][3], rely=0.5, x=x4, command=lambda: self.move_jog(text_list[2][3]))
        self.set_button_bind(self.frame_feed, text_list[2][4], rely=0.6, x=x4, command=lambda: self.move_jog(text_list[2][4]))
        self.set_button_bind(self.frame_feed, text_list[2][5], rely=0.7, x=x4, command=lambda: self.move_jog(text_list[2][5]))

    def feed_back(self):
        while True:
            if not self.global_state["connect"]: break
            self.client_feed.socket_dobot.setblocking(True)
            data = bytes()
            try:
                temp = self.client_feed.socket_dobot.recv(144000)
                if len(temp) > 1440:
                    temp = self.client_feed.socket_dobot.recv(144000)
                data = temp[0:1440]
                
                if len(data) < 1440: continue

                a = np.frombuffer(data, dtype=MyType)
                if hex((a['TestValue'][0])) == '0x123456789abcdef':
                    self.label_feed_speed["text"] = a["SpeedScaling"][0]
                    self.label_robot_mode["text"] = LABEL_ROBOT_MODE[a["RobotMode"][0]]
                    self.label_di_input["text"] = bin(a["DigitalInputs"][0])[2:].rjust(64, '0')
                    self.label_di_output["text"] = bin(a["DigitalOutputs"][0])[2:].rjust(64, '0')
                    self.set_feed_joint(LABEL_JOINT, a["QActual"])
                    self.set_feed_joint(LABEL_COORD, a["ToolVectorActual"])
                    if a["RobotMode"] == 9: self.display_error_info()
            except Exception as e:
                time.sleep(0.2)

    def display_error_info(self):
        try:
            error_info = self.client_dash.GetError("en")
            if error_info and "errMsg" in error_info and error_info["errMsg"]:
                for error in error_info["errMsg"]: self.form_error_new(error)
                return
        except Exception: pass
        
        try:
            error_list = self.client_dash.GetErrorID().split("{")[1].split("}")[0]
            error_list = json.loads(error_list)
            if error_list[0]:
                for i in error_list[0]: self.form_error(i, self.alarm_controller_dict, "Controller Error")
            for m in range(1, len(error_list)):
                if error_list[m]:
                    for n in range(len(error_list[m])): self.form_error(n, self.alarm_servo_dict, "Servo Error")
        except Exception: pass

    def form_error_new(self, error_data):
        try:
            error_info = f"Time:{error_data.get('time','N/A')} ID:{error_data.get('id','N/A')} Desc:{error_data.get('description','N/A')}\n"
            self.text_err.insert(END, error_info)
        except Exception: pass
    
    def form_error(self, index, alarm_dict: dict, type_text):
        if index in alarm_dict.keys():
            date = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            error_info = f"Time:{date} ID:{index} Type:{type_text} Desc:{alarm_dict[index]['en']['description']}\n"
            self.text_err.insert(END, error_info)

    def clear_error_info(self):
        self.text_err.delete("1.0", "end")

    def set_feed_joint(self, label, value):
        array_value = np.around(value, decimals=4)
        self.label_feed_dict[label[1][0]]["text"] = array_value[0][0]
        self.label_feed_dict[label[1][1]]["text"] = array_value[0][1]
        self.label_feed_dict[label[1][2]]["text"] = array_value[0][2]
        self.label_feed_dict[label[1][3]]["text"] = array_value[0][3]
        self.label_feed_dict[label[1][4]]["text"] = array_value[0][4]
        self.label_feed_dict[label[1][5]]["text"] = array_value[0][5]

if __name__ == "__main__":
    ui = RobotUI()
    ui.pack()
    ui.mainloop()