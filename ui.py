# -*- coding: utf-8 -*-  # 指定源文件使用 UTF-8 编码，便于正常显示中文等非 ASCII 字符
"""Tkinter 图形界面模块：提供机械臂连接、点动、运动与反馈显示等 UI 功能。

主要组件：
- RobotUI: 主界面类，封装控件搭建、事件绑定、后台反馈与控制指令。

用法：
>>> ui = RobotUI()
>>> ui.pack()
>>> ui.mainloop()
"""

# 从 threading 模块导入 Thread 类，用于创建并运行后台线程（比如实时反馈接收）
from threading import Thread

# 导入 time 模块，供时间戳、延时、格式化时间字符串等场景使用
import time

# 从 tkinter（Python 标准 GUI 库）中导入所有常用类/常量（如 Tk、Label、Button、Entry 等）
from tkinter import *

# 从 tkinter 中单独导入 ttk（主题控件库）和 messagebox（消息对话框），用于更现代的控件与弹窗提醒
from tkinter import ttk, messagebox

# 导入带滚动条的多行文本框 ScrolledText，用于日志区域和错误信息显示区域
from tkinter.scrolledtext import ScrolledText

# 从自定义的 dobot_api 模块导入所有公开符号（包括控制与反馈的 API 封装、数据结构 dtype 等）
from dobot_api import *

# 导入 json 模块，用于解析/生成 JSON 字符串（在错误回退解析时会用到）
import json

# 导入控制器报警信息列表（数据），用于根据报警 ID 查找详细说明
from files.alarmController import alarm_controller_list

# 导入伺服报警信息列表（数据），用于根据报警 ID 查找详细说明
from files.alarmServo import alarm_servo_list

# LABEL_JOINT 定义了三组与关节相关的标签/按钮文本：
# 第 0 行是六个关节的“-”点动按钮文本，
# 第 1 行是六个关节的静态显示标签（用于在 UI 上标识 J1:~J6:），
# 第 2 行是六个关节的“+”点动按钮文本。
LABEL_JOINT = [["J1-", "J2-", "J3-", "J4-", "J5-", "J6-"],
               ["J1:", "J2:", "J3:", "J4:", "J5:", "J6:"],
               ["J1+", "J2+", "J3+", "J4+", "J5+", "J6+"]]

# LABEL_COORD 定义了三组与笛卡尔坐标相关的标签/按钮文本：
# 第 0 行为 X/Y/Z/Rx/Ry/Rz 的“-”点动按钮文本，
# 第 1 行为坐标静态显示标签，
# 第 2 行为“+”点动按钮文本。
LABEL_COORD = [["X-", "Y-", "Z-", "Rx-", "Ry-", "Rz-"],
               ["X:", "Y:", "Z:", "Rx:", "Ry:", "Rz:"],
               ["X+", "Y+", "Z+", "Rx+", "Ry+", "Rz+"]]

# LABEL_ROBOT_MODE 将机器人模式编号映射为可读字符串，用于 UI 展示当前模式
LABEL_ROBOT_MODE = {
    1: "ROBOT_MODE_INIT",           # 初始化模式
    2: "ROBOT_MODE_BRAKE_OPEN",     # 机械臂刹车打开
    3: "",                          # 预留/未知（空字符串）
    4: "ROBOT_MODE_DISABLED",       # 机器人被禁用
    5: "ROBOT_MODE_ENABLE",         # 机器人已使能
    6: "ROBOT_MODE_DRAGE",          # 拖动模式（可手动拖动）
    7: "ROBOT_MODE_RUNNING",        # 正在运行
    8: "ROBOT_MODE_RECORDING",      # 示教/录制
    9: "ROBOT_MODE_ERROR",          # 错误模式
    10: "ROBOT_MODE_PAUSE",         # 暂停
    11: "ROBOT_MODE_JOG"            # 点动模式
}


# 定义 GUI 主类，封装窗口与交互逻辑
class RobotUI(object):
    """机器人 UI 主类：构建窗口、初始化控件，处理交互与后台反馈。"""

    def __init__(self):
        """初始化窗口与控件、内部状态，并准备日志/报警映射等资源。"""
        # 创建 Tk 根窗口（主窗口），后续所有控件都将放在此之下
        self.root = Tk()

        # 设置窗口标题栏文本
        self.root.title("Python demo V4")

        # 注释说明：固定窗口大小（实际上 geometry 只设置初始大小，如要禁用缩放还需 resizable(False, False)）
        # 设置窗口初始宽高为 900x850 像素
        self.root.geometry("900x850")

        # 注释说明：设置窗口图标（Windows 上常用 .ico），当前被注释，不会生效
        # self.root.iconbitmap("images/robot.ico")

        # 注释：全局状态字典，用于保存连接/使能等布尔状态或其他共享信息
        # 初始化为空字典，稍后用键 "connect"/"enable" 等存取
        self.global_state = {}

        # 注释：按钮列表，统一管理除“Connect”外的按钮，便于批量启用/禁用
        # 初始化为空列表
        self.button_list = []

        # 注释：输入框字典，按“标签文本”作为键保存 Entry 引用，便于后续读取数值
        # 例如：self.entry_dict["X:"] 即对应 X 坐标输入框
        self.entry_dict = {}

        # 注释：“Robot Connect” 分组框，承载 IP、端口和连接按钮
        # LabelFrame 是带标题的容器；labelanchor 指定标题位置，bg 背景色，width/height 期望尺寸，border 设边框宽（建议用 bd）
        self.frame_robot = LabelFrame(self.root, text="Robot Connect",
                                      labelanchor="nw", bg="#FFFFFF", width=870, height=120, border=2)

        # 创建标签：提示用户输入 IP 地址
        self.label_ip = Label(self.frame_robot, text="IP Address:")

        # 使用 place 绝对/相对混合布局：rely=0.2 表示相对父容器高度的 20% 处，x=10 表示 X 方向偏移 10 像素
        self.label_ip.place(rely=0.2, x=10)

        # 创建一个 Tk 字符串变量，初始值为默认 IP。与 Entry 的 textvariable 绑定可实现双向同步
        ip_port = StringVar(self.root, value="111.111.130.198")

        # 创建 IP 输入框，设置宽度为 12（字符数），绑定到 ip_port 变量
        self.entry_ip = Entry(self.frame_robot, width=12, textvariable=ip_port)

        # 放置 IP 输入框到同一行，X 方向偏移 90 像素
        self.entry_ip.place(rely=0.2, x=90)

        # 创建标签：提示用户输入 Dashboard 端口
        self.label_dash = Label(self.frame_robot, text="Dashboard Port:")

        # 放置该标签到同一行，X 偏移 210 像素
        self.label_dash.place(rely=0.2, x=210)

        # 创建一个 Tk 整型变量，默认值 29999，作为 Dashboard 端口
        dash_port = IntVar(self.root, value=29999)

        # 创建 Dashboard 端口输入框，宽度 7，绑定 dash_port 变量
        self.entry_dash = Entry(
            self.frame_robot, width=7, textvariable=dash_port)

        # 放置该输入框到同一行，X 偏移 320 像素
        self.entry_dash.place(rely=0.2, x=320)

        # 创建标签：提示用户输入 Feedback 端口
        self.label_feed = Label(self.frame_robot, text="Feedback Port:")

        # 放置该标签到同一行，X 偏移 420 像素
        self.label_feed.place(rely=0.2, x=420)

        # 创建一个 Tk 整型变量，默认值 30004，作为 Feedback 端口
        feed_port = IntVar(self.root, value=30004)

        # 创建 Feedback 端口输入框，宽度 7，绑定 feed_port 变量
        self.entry_feed = Entry(
            self.frame_robot, width=7, textvariable=feed_port)

        # 放置该输入框到同一行，X 偏移 520 像素
        self.entry_feed.place(rely=0.2, x=520)

        # 创建“Connect”按钮（或断开），点击后调用 self.connect_port 函数尝试连接/断开
        self.button_connect = self.set_button(master=self.frame_robot,
                                              text="Connect", rely=0.6, x=630, command=self.connect_port)

        # 设置按钮的显示宽度为 10（字符）
        self.button_connect["width"] = 10

        # 在全局状态中记录当前连接状态，False 表示未连接
        self.global_state["connect"] = False

        # 创建“Dashboard Function” 分组框，承载使能、清错、速度、DO 输出设置等控件
        self.frame_dashboard = LabelFrame(self.root, text="Dashboard Function",
                                          labelanchor="nw", bg="#FFFFFF", pady=10, width=870, height=120, border=2)

        # 创建“Enable”按钮，点击时调用 self.enable 切换使能/禁用
        self.button_enable = self.set_button(master=self.frame_dashboard,
                                             text="Enable", rely=0.1, x=10, command=self.enable)

        # 设置“Enable”按钮的显示宽度
        self.button_enable["width"] = 7

        # 新增：启动拖拽模式按钮
        self.button_start_drag = self.set_button(master=self.frame_dashboard,
                                                 text="StartDrag", rely=0.1, x=100, command=self.start_drag)
        self.button_start_drag["width"] = 10

        # 在全局状态中记录当前使能状态，初始为未使能 False
        self.global_state["enable"] = False
        # 拖拽模式开关状态
        self.global_state["drag"] = False

        # 拖拽灵敏度设置 UI（轴与数值）
        self.label_drag_sen = Label(self.frame_dashboard, text="Drag Sensitivity:")
        self.label_drag_sen.place(rely=0.55, x=430)
        self.label_drag_axis = Label(self.frame_dashboard, text="Axis:")
        self.label_drag_axis.place(rely=0.55, x=550)
        self.spin_drag_axis = Spinbox(self.frame_dashboard, from_=0, to=6, width=4)
        self.spin_drag_axis.place(rely=0.55, x=590)
        self.label_drag_val = Label(self.frame_dashboard, text="Value:")
        self.label_drag_val.place(rely=0.55, x=640)
        self.spin_drag_value = Spinbox(self.frame_dashboard, from_=1, to=90, width=5)
        self.spin_drag_value.place(rely=0.55, x=690)
        self.button_drag_apply = self.set_button(master=self.frame_dashboard,
                                                 text="Apply", rely=0.5, x=750, command=self.set_drag_sensitivity)

        # 在全局状态中记录当前使能状态，初始为未使能 False
        self.global_state["enable"] = False

        # 创建“ClearError”按钮，点击时调用 self.clear_error 清除报警/错误
        self.set_button(master=self.frame_dashboard,
                        text="ClearError", rely=0.1, x=200, command=self.clear_error)

        # 创建标签“Speed Ratio:”，提示用户设置全局速度比例（百分比）
        self.label_speed = Label(self.frame_dashboard, text="Speed Ratio:")

        # 放置速度标签到同一行，靠右一些
        self.label_speed.place(rely=0.1, x=430)

        # 创建一个 Tk 字符串变量，默认值 "50"（表示 50%）
        s_value = StringVar(self.root, value="50")

        # 创建速度比例输入框，宽度 6，绑定 s_value 变量
        self.entry_speed = Entry(self.frame_dashboard,
                                 width=6, textvariable=s_value)

        # 放置速度输入框
        self.entry_speed.place(rely=0.1, x=520)

        # 创建“%”符号标签，表示单位为百分比
        self.label_cent = Label(self.frame_dashboard, text="%")

        # 放置“%”标签
        self.label_cent.place(rely=0.1, x=550)

        # 创建“Confirm”按钮，点击时调用 self.confirm_speed 下发速度比例设置
        self.set_button(master=self.frame_dashboard,
                        text="Confirm", rely=0.1, x=586, command=self.confirm_speed)

        # 创建标签“Digital Outputs: Index:”，用于设置 DO（数字输出）的索引
        self.label_digitial = Label(
            self.frame_dashboard, text="Digital Outputs: Index:")

        # 放置 DO 索引提示标签
        self.label_digitial.place(rely=0.55, x=10)

        # 创建一个 Tk 整型变量，默认值 "1"（注意字符串形式也可接受）
        i_value = IntVar(self.root, value="1")

        # 创建 DO 索引输入框，宽度 5，绑定 i_value 变量
        self.entry_index = Entry(
            self.frame_dashboard, width=5, textvariable=i_value)

        # 放置 DO 索引输入框
        self.entry_index.place(rely=0.55, x=160)

        # 创建“Status:”标签，提示选择 DO 的状态（On/Off）
        self.label_status = Label(self.frame_dashboard, text="Status:")

        # 放置状态标签
        self.label_status.place(rely=0.55, x=220)

        # 创建下拉框 Combobox（来自 ttk），用于选择 DO 状态
        self.combo_status = ttk.Combobox(self.frame_dashboard, width=5)

        # 设置下拉框的可选值为 On/Off
        self.combo_status["value"] = ("On", "Off")

        # 设置默认选中第 0 项（On）
        self.combo_status.current(0)

        # 设置下拉框为只读，禁止用户手动输入其他文本
        self.combo_status["state"] = "readonly"

        # 放置下拉框
        self.combo_status.place(rely=0.55, x=275)

        # 创建“Confirm”按钮，点击时调用 self.confirm_do 下发 DO 输出命令
        self.set_button(self.frame_dashboard, "Confirm",
                        rely=0.55, x=350, command=self.confirm_do)

        # 创建“Move Function” 分组框，承载位置输入与运动按钮
        self.frame_move = LabelFrame(self.root, text="Move Function", labelanchor="nw",
                                     bg="#FFFFFF", width=870, pady=10, height=130, border=2)

        # 依次创建笛卡尔坐标系的六个输入：X/Y/Z/Rx/Ry/Rz，设置默认值并登记到 entry_dict
        self.set_move(text="X:", label_value=10,
                      default_value="600", entry_value=40, rely=0.1, master=self.frame_move)

        self.set_move(text="Y:", label_value=110,
                      default_value="-260", entry_value=140, rely=0.1, master=self.frame_move)

        self.set_move(text="Z:", label_value=210,
                      default_value="380", entry_value=240, rely=0.1, master=self.frame_move)

        self.set_move(text="Rx:", label_value=310,
                      default_value="170", entry_value=340, rely=0.1, master=self.frame_move)

        self.set_move(text="Ry:", label_value=410,
                      default_value="12", entry_value=440, rely=0.1, master=self.frame_move)

        self.set_move(text="Rz:", label_value=510,
                      default_value="140", entry_value=540, rely=0.1, master=self.frame_move)

        # 创建笛卡尔空间的 MovJ 按钮（关节插补到达目标位姿），点击调用 self.movj
        self.set_button(master=self.frame_move, text="MovJ",
                        rely=0.05, x=610, command=self.movj)

        # 创建笛卡尔空间的 MovL 按钮（直线插补到达目标位姿），点击调用 self.movl
        self.set_button(master=self.frame_move, text="MovL",
                        rely=0.05, x=700, command=self.movl)

        # 依次创建关节空间的六个输入：J1~J6，设置默认角度并登记
        self.set_move(text="J1:", label_value=10,
                      default_value="0", entry_value=40, rely=0.5, master=self.frame_move)

        self.set_move(text="J2:", label_value=110,
                      default_value="-20", entry_value=140, rely=0.5, master=self.frame_move)

        self.set_move(text="J3:", label_value=210,
                      default_value="-80", entry_value=240, rely=0.5, master=self.frame_move)

        self.set_move(text="J4:", label_value=310,
                      default_value="30", entry_value=340, rely=0.5, master=self.frame_move)

        self.set_move(text="J5:", label_value=410,
                      default_value="90", entry_value=440, rely=0.5, master=self.frame_move)

        self.set_move(text="J6:", label_value=510,
                      default_value="120", entry_value=540, rely=0.5, master=self.frame_move)

        # 创建关节空间的 MovJ 按钮（按关节角到达目标），点击调用 self.joint_movj
        self.set_button(master=self.frame_move,
                        text="MovJ", rely=0.45, x=610, command=self.joint_movj)

        # 创建一个普通 Frame 作为左侧反馈和右侧日志的容器
        self.frame_feed_log = Frame(
            self.root, bg="#FFFFFF", width=870, pady=10, height=400, border=2)

        # 注释：Feedback 小分区（左侧），用于实时数据显示
        self.frame_feed = LabelFrame(self.frame_feed_log, text="Feedback", labelanchor="nw",
                                     bg="#FFFFFF", width=550, height=150)

        # 将 Feedback 分区使用 place 放入父容器左侧，占据整个高度
        self.frame_feed.place(relx=0, rely=0, relheight=1)

        # 创建静态标签，显示“Current Speed Ratio:”作为当前速度比例说明
        self.set_label(self.frame_feed,
                       text="Current Speed Ratio:", rely=0.05, x=10)

        # 创建一个动态标签，用于实时显示速度比例的数值内容
        self.label_feed_speed = self.set_label(
            self.frame_feed, "", rely=0.05, x=145)

        # 创建一个静态“%”标签，表示速度比例单位为百分比
        self.set_label(self.frame_feed, text="%", rely=0.05, x=175)

        # 创建静态标签，显示“Robot Mode:”作为当前机器人模式说明
        self.set_label(self.frame_feed, text="Robot Mode:", rely=0.1, x=10)

        # 创建一个动态标签，用于实时显示机器人模式名称（通过映射 LABEL_ROBOT_MODE）
        self.label_robot_mode = self.set_label(
            self.frame_feed, "", rely=0.1, x=95)

        # 点动按钮和实时坐标显示的字典，后续将存放每个“X:/J1:”标签对应的值显示 Label 引用
        self.label_feed_dict = {}

        # 为关节（J1~J6）创建“- / 标签 / +”三列与对应数值显示
        self.set_feed(LABEL_JOINT, 9, 52, 74, 117)

        # 为笛卡尔坐标（X~Rz）创建“- / 标签 / +”三列与对应数值显示
        self.set_feed(LABEL_COORD, 165, 209, 231, 272)

        # 创建“Digital I/O” 显示区域的静态标签：Digital Inputs
        self.set_label(self.frame_feed, "Digital Inputs:", rely=0.8, x=11)

        # 创建动态标签用于显示数字输入（以 64 位二进制字符串形式展示）
        self.label_di_input = self.set_label(
            self.frame_feed, "", rely=0.8, x=100)

        # 创建“Digital Outputs” 静态标签
        self.set_label(self.frame_feed, "Digital Outputs:", rely=0.85, x=10)

        # 创建动态标签用于显示数字输出（以 64 位二进制字符串形式展示）
        self.label_di_output = self.set_label(
            self.frame_feed, "", rely=0.85, x=100)

        # 创建“Error Info” 分组框，用于显示错误/报警的详细文本
        self.frame_err = LabelFrame(self.frame_feed, text="Error Info", labelanchor="nw",
                                    bg="#FFFFFF", width=180, height=50)

        # 放置错误信息分组框在 Feedback 区域的右侧，并设置高度占比
        self.frame_err.place(relx=0.65, rely=0, relheight=0.7)

        # 在错误信息分组中创建一个带滚动条的文本框，用于累计展示错误内容
        self.text_err = ScrolledText(
            self.frame_err, width=170, height=50, relief="flat")

        # 放置错误文本框，占满子区域
        self.text_err.place(rely=0, relx=0, relheight=0.7, relwidth=1)

        # 创建一个“Clear”按钮，点击时清空错误显示文本框
        self.set_button(self.frame_feed, "Clear", rely=0.71,
                        x=487, command=self.clear_error_info)

        # 创建“Log” 分组框（右侧），用于显示日志信息（比如连接/接收状态）
        self.frame_log = LabelFrame(self.frame_feed_log, text="Log", labelanchor="nw",
                                    bg="#FFFFFF", width=300, height=150)

        # 放置日志分组框在右侧，填满父容器高度
        self.frame_log.place(relx=0.65, rely=0, relheight=1)

        # 在日志分组中创建带滚动条的文本框，作为日志输出展示区域
        self.text_log = ScrolledText(
            self.frame_log, width=270, height=140, relief="flat")

        # 放置日志文本框，占满整个日志分组框
        self.text_log.place(rely=0, relx=0, relheight=1, relwidth=1)

        # 初始化 Dashboard/Feedback 客户端句柄为 None，表示尚未连接
        self.client_dash = None

        self.client_feed = None

        # 将报警列表转换为以 id 为键的字典，便于快速查找控制器报警信息
        self.alarm_controller_dict = self.convert_dict(alarm_controller_list)

        # 将报警列表转换为以 id 为键的字典，便于快速查找伺服报警信息
        self.alarm_servo_dict = self.convert_dict(alarm_servo_list)

    def convert_dict(self, alarm_list):
        """将报警列表转换为以 id 为键的字典。
        
        参数:
            alarm_list (list[dict]): 报警条目列表，每项包含 id 等字段。
        返回:
            dict: {id: alarm_item} 的映射，便于 O(1) 查询。
        """
        # 初始化空字典
        alarm_dict = {}

        # 遍历列表里的每个报警对象 i
        for i in alarm_list:
            # 以报警对象的 "id" 字段为键，整条对象为值，存入字典
            alarm_dict[i["id"]] = i

        # 返回构建好的映射字典
        return alarm_dict

    def read_file(self, path):
        """读取 JSON 文件并解析为 Python 对象。
        
        参数:
            path (str): 文件路径。
        返回:
            Any: 解析后的字典/列表内容。
        异常:
            FileNotFoundError/JSONDecodeError: 当路径不存在或内容非法时抛出。
        """
        # 注释：读 json 文件相对耗时，因此实际维护在内存的 alarm_controller_list / alarm_servo_list
        # self.read_file("files/alarm_controller.json")  # 示例（已注释）
        # 以 UTF-8 编码打开文件，读取其内容
        with open(path, "r", encoding="utf8") as fp:
            # 使用 json.load 将文件内容解析为 Python 数据结构（字典/列表）
            json_data = json.load(fp)

        # 返回解析结果
        return json_data

    def mainloop(self):
        """启动 Tk 事件循环，进入阻塞式 GUI 运行状态。"""
        # 进入主循环（阻塞），处理 GUI 事件（点击、输入等）
        self.root.mainloop()

    def pack(self):
        """将各主分区容器按垂直顺序布局到主窗口。"""
        # 把“Robot Connect” 分组放入窗口（垂直堆叠）
        self.frame_robot.pack()

        # 把“Dashboard Function” 分组放入窗口
        self.frame_dashboard.pack()

        # 把“Move Function” 分组放入窗口
        self.frame_move.pack()

        # 把“Feedback/Log” 容器放入窗口
        self.frame_feed_log.pack()

    def set_move(self, text, label_value, default_value, entry_value, rely, master):
        """创建一组“标签+输入框”并登记至 entry_dict。
        
        参数:
            text (str): 标签文本，如 "X:"/"J1:"。
            label_value (int): 标签放置的 x 坐标。
            default_value (str): 输入框默认值（字符串）。
            entry_value (int): 输入框放置的 x 坐标。
            rely (float): 相对纵向位置 (0~1)。
            master (tk.Widget): 父容器。
        返回:
            None
        """
        # 创建说明标签（如 "X:"、"J1:"），放在 master 容器内
        self.label = Label(master, text=text)

        # 使用 place 放置标签到指定相对高度和 X 位置
        self.label.place(rely=rely, x=label_value)

        # 创建一个 Tk 字符串变量，初始值为 default_value（显示在 Entry 中）
        value = StringVar(self.root, value=default_value)

        # 创建输入框 Entry，宽度为 6 个字符，并绑定到 value 变量
        self.entry_temp = Entry(master, width=6, textvariable=value)

        # 放置该输入框到指定位置
        self.entry_temp.place(rely=rely, x=entry_value)

        # 将输入框对象按其文本键（text）登记到字典，便于后续读取该输入框的值
        self.entry_dict[text] = self.entry_temp

    def move_jog(self, text):
        """开始点动（按住时调用）。
        
        参数:
            text (str): 方向字符串，如 "J1-"、"X+"、"Ry-" 等。
        行为:
            - 关节点动: 直接 MoveJog(text)
            - 笛卡尔点动: MoveJog(text, coordtype=1, user=0, tool=0)
        前置条件:
            已连接 self.global_state["connect"] 为 True。
        """
        # 只有在已连接的情况下才允许发送点动命令
        if self.global_state["connect"]:
            # 如果第一个字符是 "J"，代表是关节点动（J1~J6）
            if text[0] == "J":
                # 调用 Dashboard 的 MoveJog 接口，传入如 "J1-" 或 "J2+" 的指令
                self.client_dash.MoveJog(text)
            else:
                # 否则是笛卡尔坐标点动（X/Y/Z/Rx/Ry/Rz），需要指定坐标类型、用户坐标与工具坐标等参数
                self.client_dash.MoveJog(text,coordtype=1,user=0,tool=0)

    def move_stop(self, event):
        """停止点动（松开时调用）。
        
        参数:
            event (tk.Event): 鼠标释放事件对象（未使用）。
        行为:
            发送 MoveJog("")，使控制器立即停止当前点动。
        """
        # 仅当已连接时才发送停止命令
        if self.global_state["connect"]:
            # 发送 MoveJog("") 表示立即停止点动
            self.client_dash.MoveJog("")

    def set_button(self, master, text, rely, x, **kargs):
        """创建普通按钮并放置到指定位置。
        
        参数:
            master (tk.Widget): 父容器。
            text (str): 按钮文本。
            rely (float): 相对纵向位置 (0~1)。
            x (int): x 坐标。
            **kargs: 需包含 command=Callable 的回调。
        返回:
            tkinter.Button: 创建的按钮实例。
        备注:
            除 "Connect" 外的按钮会在连接前置为禁用状态。
        """
        # 创建 Button 控件，显示文本 text，并将 command 作为点击回调函数
        self.button = Button(master, text=text, padx=5,
                             command=kargs["command"])

        # 使用 place 放置按钮到指定位置
        self.button.place(rely=rely, x=x)

        # 如果不是“Connect”按钮，则先禁用，并放入 button_list 统一管理
        if text != "Connect":
            # 设置按钮状态为 "disable"（不可点击）
            self.button["state"] = "disable"
            # 保存到列表中，后续可统一启用/禁用
            self.button_list.append(self.button)
        # 返回按钮对象引用，以便外部进一步设置属性
        return self.button

    def set_button_bind(self, master, text, rely, x, **kargs):
        """创建点动类按钮：按下开始点动，松开停止。
        
        参数:
            master (tk.Widget): 父容器。
            text (str): 按钮文本（亦作为点动方向）。
            rely (float): 相对纵向位置 (0~1)。
            x (int): x 坐标。
            **kargs: 预留参数，未使用。
        返回:
            tkinter.Button: 创建的按钮实例。
        """
        # 创建 Button 控件，显示文本 text
        self.button = Button(master, text=text, padx=5)

        # 绑定鼠标左键按下事件：调用 move_jog 开始点动，传入当前按钮的文本 text 作为方向
        self.button.bind("<ButtonPress-1>",
                         lambda event: self.move_jog(text=text))

        # 绑定鼠标左键释放事件：调用 move_stop 停止点动
        self.button.bind("<ButtonRelease-1>", self.move_stop)

        # 使用 place 放置点动按钮
        self.button.place(rely=rely, x=x)

        # 非“Connect”按钮默认禁用，待连接成功后统一启用
        if text != "Connect":
            # 设置按钮状态为禁用
            self.button["state"] = "disable"
            # 保存按钮到列表
            self.button_list.append(self.button)
        # 返回按钮引用
        return self.button

    def set_label(self, master, text, rely, x):
        """创建并放置标签。
        
        参数:
            master (tk.Widget): 父容器。
            text (str): 标签文本。
            rely (float): 相对纵向位置 (0~1)。
            x (int): x 坐标。
        返回:
            tkinter.Label: 创建的标签实例。
        """
        # 创建 Label 控件，显示指定文本
        self.label = Label(master, text=text)

        # 使用 place 放置标签
        self.label.place(rely=rely, x=x)

        # 返回标签引用，便于外部动态修改其 text
        return self.label

    def connect_port(self):
        """连接/断开控制器与反馈端口，并联动启用/禁用其他按钮与反馈线程。
        
        行为:
            - 已连接时：关闭连接并禁用功能按钮；
            - 未连接时：按输入的 IP/端口建立连接，启用功能按钮并启动反馈线程。
        可能抛出:
            异常在连接失败时通过 messagebox 弹窗提示。
        """
        # 如果当前已连接，则走断开逻辑
        if self.global_state["connect"]:
            # 打印断开成功（调试/日志用途）
            print("断开成功")
            # 关闭 Dashboard 客户端连接（内部会关闭 socket 等资源）
            self.client_dash.close()
            # 关闭 Feedback 客户端连接
            self.client_feed.close()
            # 置空句柄，表示不再可用
            self.client_dash = None
            self.client_feed = None

            # 遍历所有非 Connect 按钮，将其禁用，防止误操作
            for i in self.button_list:
                i["state"] = "disable"
            # 将连接按钮文本改回 "Connect"
            self.button_connect["text"] = "Connect"
            # 重置拖拽状态与按钮外观
            self.global_state["drag"] = False
            try:
                self.button_start_drag["relief"] = "raised"
                self.button_start_drag["text"] = "StartDrag"
            except Exception:
                pass
        else:
            # 若当前未连接，则尝试建立连接
            try:
                # 打印连接成功（仅作为提示，实际连接在下一行）
                print("连接成功")
                # 创建 Dashboard 客户端对象，传入 IP、端口与日志文本框（用于输出文本日志）
                self.client_dash = DobotApiDashboard(
                    self.entry_ip.get(), int(self.entry_dash.get()), self.text_log)
                # 创建 Feedback 客户端对象，传入 IP、端口与日志文本框
                self.client_feed = DobotApiFeedBack(
                    self.entry_ip.get(), int(self.entry_feed.get()), self.text_log)
            except Exception as e:
                # 弹出错误消息框，展示连接异常信息，然后返回不再继续
                messagebox.showerror("Attention!", f"Connection Error:{e}")
                return

            # 连接成功后，遍历按钮列表，将其设为可用状态
            for i in self.button_list:
                i["state"] = "normal"
            # 将连接按钮文本改为 "Disconnect"，提示再次点击会断开
            self.button_connect["text"] = "Disconnect"
        # 翻转连接状态布尔值（False->True）
        self.global_state["connect"] = not self.global_state["connect"]
        # 连接建立后，启动反馈线程以接收并刷新实时数据
        self.set_feed_back()

    def set_feed_back(self):
        """在已连接状态下启动后台线程，持续接收并刷新反馈数据。"""
        # 仅当当前为已连接状态才启动线程
        if self.global_state["connect"]:
            # 创建线程对象，目标函数为 self.feed_back（无限循环接收）
            thread = Thread(target=self.feed_back)
            # 将线程设置为守护线程（主程序退出时无需等此线程）
            thread.setDaemon(True)
            # 启动线程开始运行
            thread.start()

    def enable(self):
        """切换机器人使能/禁用状态，并更新按钮文本。
        
        行为:
            - 当未使能 -> 调用 EnableRobot()
            - 当已使能 -> 调用 DisableRobot()
        """
        # 如果当前处于已使能状态，则执行禁用
        if self.global_state["enable"]:
            # 调用 Dashboard 的 DisableRobot 使机器人下电/禁用
            self.client_dash.DisableRobot()
            # 将按钮文本设为 "Enable"，表示点击可以使能
            self.button_enable["text"] = "Enable"
        else:
            # 如果当前未使能，则执行使能操作
            self.client_dash.EnableRobot()
            # 注释：必要时可以适当 sleep 等待硬件响应
            # time.sleep(0.5)
            # 将按钮文本设为 "Disable"，表示点击可以禁用
            self.button_enable["text"] = "Disable"

        # 翻转使能状态布尔值（True<->False）
        self.global_state["enable"] = not self.global_state["enable"]

    def start_drag(self):
        """切换拖拽模式（StartDrag/StopDrag），并更新按钮外观。"""
        if not self.global_state.get("connect"):
            return
        # 切换状态
        if not self.global_state.get("drag", False):
            # 进入拖拽模式
            try:
                self.client_dash.StartDrag()
            except Exception:
                return
            self.global_state["drag"] = True
            # 按钮呈现按下外观
            self.button_start_drag["relief"] = "sunken"
            self.button_start_drag["text"] = "StopDrag"
        else:
            # 退出拖拽模式
            try:
                self.client_dash.StopDrag()
            except Exception:
                return
            self.global_state["drag"] = False
            # 恢复按钮外观
            self.button_start_drag["relief"] = "raised"
            self.button_start_drag["text"] = "StartDrag"

    def set_drag_sensitivity(self):
        """设置拖拽灵敏度（DragSensivity）。"""
        try:
            axis = int(self.spin_drag_axis.get())
            value = int(self.spin_drag_value.get())
            self.client_dash.DragSensivity(axis, value)
        except Exception:
            pass

    def clear_error(self):
        """清除控制器当前报警（Dashboard.ClearError）。"""
        # 调用 Dashboard 的 ClearError 接口
        self.client_dash.ClearError()

    def confirm_speed(self):
        """读取速度比例输入并调用 SpeedFactor 下发到控制器。
        
        输入:
            使用 entry_speed 的数值（百分比 0~100）。
        """
        # 将 Entry 中的字符串转为 int 作为 SpeedFactor 值（百分比），调用接口设置
        self.client_dash.SpeedFactor(int(self.entry_speed.get()))

    def movj(self):
        """以关节插补方式运动至目标位姿（输入为笛卡尔位姿）。
        
        数据来源:
            entry_dict["X:"/"Y:"/"Z:"/"Rx:"/"Ry:"/"Rz:"]
        调用:
            Dashboard.MovJ(..., coordinateMode=0)
        """
        # 逐个从 entry_dict 读取字符串并转为 float，最后一个参数 0 表示笛卡尔模式
        self.client_dash.MovJ(float(self.entry_dict["X:"].get()), float(self.entry_dict["Y:"].get()), float(self.entry_dict["Z:"].get()),
                              float(self.entry_dict["Rx:"].get()), float(self.entry_dict["Ry:"].get()), float(self.entry_dict["Rz:"].get()),0)

    def movl(self):
        """以直线插补方式运动至目标位姿（输入为笛卡尔位姿）。
        
        数据来源:
            entry_dict["X:"/"Y:"/"Z:"/"Rx:"/"Ry:"/"Rz:"]
        调用:
            Dashboard.MovL(..., coordinateMode=0)
        """
        # 逐个从 entry_dict 读取字符串并转为 float，最后一个参数 0 表示笛卡尔模式
        self.client_dash.MovL(float(self.entry_dict["X:"].get()), float(self.entry_dict["Y:"].get()), float(self.entry_dict["Z:"].get()),
                              float(self.entry_dict["Rx:"].get()), float(self.entry_dict["Ry:"].get()), float(self.entry_dict["Rz:"].get()),0)

    def joint_movj(self):
        """以关节插补方式运动至目标关节角（输入为 J1~J6）。
        
        数据来源:
            entry_dict["J1:" ~ "J6:"]
        调用:
            Dashboard.MovJ(..., coordinateMode=1)
        """
        # 逐个从 entry_dict 读取关节角字符串并转为 float，最后一个参数 1 表示关节模式
        self.client_dash.MovJ(float(self.entry_dict["J1:"].get()), float(self.entry_dict["J2:"].get()), float(self.entry_dict["J3:"].get()),
                                   float(self.entry_dict["J4:"].get()), float(self.entry_dict["J5:"].get()), float(self.entry_dict["J6:"].get()),1)

    def confirm_do(self):
        """根据下拉框选择设置 DO（数字输出）状态。
        
        输入:
            entry_index: DO 索引；combo_status: On=1/Off=0
        调用:
            Dashboard.DO(index, value)
        """
        # 如果下拉框选择 "On"，则输出高电平 1
        if self.combo_status.get() == "On":
            # 打印中文提示“高电平”方便调试
            print("高电平")
            # 调用 Dashboard.DO 接口，传入索引与 1（高电平）
            self.client_dash.DO(int(self.entry_index.get()), 1)
        else:
            # 否则输出低电平 0
            print("低电平")
            # 调用 Dashboard.DO 接口，传入索引与 0（低电平）
            self.client_dash.DO(int(self.entry_index.get()), 0)

    def set_feed(self, text_list, x1, x2, x3, x4):
        """批量创建点动按钮与数值显示标签（关节或笛卡尔）。
        
        参数:
            text_list (list[list[str]]): 三行文本矩阵，[减号行, 标签行, 加号行]。
            x1 (int): 减号按钮列 x 坐标。
            x2 (int): 标签列 x 坐标。
            x3 (int): 数值显示列 x 坐标。
            x4 (int): 加号按钮列 x 坐标。
        返回:
            None
        """
        # 创建 6 个“-”点动按钮（如 J1-、J2- 或 X-、Y-），按住即移动，对应列 X=x1
        self.set_button_bind(
            self.frame_feed, text_list[0][0], rely=0.2, x=x1, command=lambda: self.move_jog(text_list[0][0]))
        self.set_button_bind(
            self.frame_feed, text_list[0][1], rely=0.3, x=x1, command=lambda: self.move_jog(text_list[0][1]))
        self.set_button_bind(
            self.frame_feed, text_list[0][2], rely=0.4, x=x1, command=lambda: self.move_jog(text_list[0][2]))
        self.set_button_bind(
            self.frame_feed, text_list[0][3], rely=0.5, x=x1, command=lambda: self.move_jog(text_list[0][3]))
        self.set_button_bind(
            self.frame_feed, text_list[0][4], rely=0.6, x=x1, command=lambda: self.move_jog(text_list[0][4]))
        self.set_button_bind(
            self.frame_feed, text_list[0][5], rely=0.7, x=x1, command=lambda: self.move_jog(text_list[0][5]))

        # 创建 6 个静态标签（如 J1:、J2: 或 X:、Y:），用于标识对应的值
        self.set_label(self.frame_feed, text_list[1][0], rely=0.21, x=x2)
        self.set_label(self.frame_feed, text_list[1][1], rely=0.31, x=x2)
        self.set_label(self.frame_feed, text_list[1][2], rely=0.41, x=x2)
        self.set_label(self.frame_feed, text_list[1][3], rely=0.51, x=x2)
        self.set_label(self.frame_feed, text_list[1][4], rely=0.61, x=x2)
        self.set_label(self.frame_feed, text_list[1][5], rely=0.71, x=x2)

        # 为每个标签创建一个动态值显示 Label，并把引用保存在字典 label_feed_dict 中，键为标签文本（如 "J1:" 或 "X:"）
        self.label_feed_dict[text_list[1][0]] = self.set_label(
            self.frame_feed, " ", rely=0.21, x=x3)
        self.label_feed_dict[text_list[1][1]] = self.set_label(
            self.frame_feed, " ", rely=0.31, x=x3)
        self.label_feed_dict[text_list[1][2]] = self.set_label(
            self.frame_feed, " ", rely=0.41, x=x3)
        self.label_feed_dict[text_list[1][3]] = self.set_label(
            self.frame_feed, " ", rely=0.51, x=x3)
        self.label_feed_dict[text_list[1][4]] = self.set_label(
            self.frame_feed, " ", rely=0.61, x=x3)
        self.label_feed_dict[text_list[1][5]] = self.set_label(
            self.frame_feed, " ", rely=0.71, x=x3)

        # 创建 6 个“+”点动按钮（如 J1+ 或 X+），按住移动正方向，对应列 X=x4
        self.set_button_bind(
            self.frame_feed, text_list[2][0], rely=0.2, x=x4, command=lambda: self.move_jog(text_list[2][0]))
        self.set_button_bind(
            self.frame_feed, text_list[2][1], rely=0.3, x=x4, command=lambda: self.move_jog(text_list[2][0]))
        self.set_button_bind(
            self.frame_feed, text_list[2][2], rely=0.4, x=x4, command=lambda: self.move_jog(text_list[2][0]))
        self.set_button_bind(
            self.frame_feed, text_list[2][3], rely=0.5, x=x4, command=lambda: self.move_jog(text_list[2][0]))
        self.set_button_bind(
            self.frame_feed, text_list[2][4], rely=0.6, x=x4, command=lambda: self.move_jog(text_list[2][0]))
        self.set_button_bind(
            self.frame_feed, text_list[2][5], rely=0.7, x=x4, command=lambda: self.move_jog(text_list[2][0]))

    def feed_back(self):
        """后台反馈线程：循环接收解析数据并刷新 UI。
        
        行为:
            - 读取固定长度数据帧，验证魔数后更新速度、模式、IO、位姿/关节等。
            - 若模式为错误，调用 display_error_info 展示详细报警。
        退出条件:
            self.global_state["connect"] 变为 False。
        """
        # 无限循环直到检测到断开连接
        while True:
            # 打印当前连接状态，便于调试观察线程行为
            print("self.global_state(connect)", self.global_state["connect"])
            # 如果已经断开连接，则退出循环结束线程
            if not self.global_state["connect"]:
                break

            # 将反馈 socket 设置为阻塞模式（recv 会等待直到有数据到达）
            self.client_feed.socket_dobot.setblocking(True)  # 设置为阻塞模式
            # 初始化空字节串，用于拼接/存放收到的数据
            data = bytes()
            # 从 socket 接收最多 144000 字节数据（具体上限与协议实现相关）
            temp = self.client_feed.socket_dobot.recv(144000)
            # 如果第一轮收到的数据长度大于 1440，则再收一次（用于规避黏包等情况）
            if len(temp) > 1440:
                temp = self.client_feed.socket_dobot.recv(144000)
            # 仅取前 1440 字节作为一个完整的数据帧（固定长度解析）
            data = temp[0:1440]
        

            # 使用 numpy.frombuffer 按 dtype=MyType 的结构体定义将二进制缓冲区解析为结构化数组
            a = np.frombuffer(data, dtype=MyType)
            # 打印解析出的 RobotMode 的第一个元素（标量），用于调试
            print("robot_mode:", a["RobotMode"][0])
            # 打印 TestValue 字段的十六进制形式，用于校验数据包正确性（魔数）
            print("TestValue:", hex((a['TestValue'][0])))
            # 若魔数匹配 0x123456789abcdef，认为本帧有效，继续刷新 UI
            if hex((a['TestValue'][0])) == '0x123456789abcdef':
                # 以下为可选调试输出，已注释：打印工具端位姿与实际关节角
                # print('tool_vector_actual',
                #       np.around(a['tool_vector_actual'], decimals=4))
                # print('QActual', np.around(a['q_aQActualctual'], decimals=4))

                # 刷新界面上的“当前速度比例”数值标签
                self.label_feed_speed["text"] = a["SpeedScaling"][0]
                # 读取 RobotMode 的编号并映射为可读字符串，显示在模式标签上
                self.label_robot_mode["text"] = LABEL_ROBOT_MODE[a["RobotMode"][0]]
                # 将数字输入转为二进制字符串（去掉前缀 0b），再左侧补零到 64 位，显示在标签上
                self.label_di_input["text"] = bin(a["DigitalInputs"][0])[\
                    2:].rjust(64, '0')
                # 将数字输出同样转为 64 位二进制字符串显示
                self.label_di_output["text"] = bin(a["DigitalOutputs"][0])[\
                    2:].rjust(64, '0')

                # 刷新 6 个关节的实时关节角显示（QActual）
                self.set_feed_joint(LABEL_JOINT, a["QActual"])
                # 刷新 6 个笛卡尔的实时位姿显示（ToolVectorActual）
                self.set_feed_joint(LABEL_COORD, a["ToolVectorActual"])

                # 实时记录六关节角到日志区域
                try:
                    joint_vals = [float(a["QActual"][0][i]) for i in range(6)]
                    ts = time.strftime("%H:%M:%S", time.localtime())
                    self.text_log.insert(END, f"[{ts}] J: {', '.join(['{:.4f}'.format(v) for v in joint_vals])}\n")
                    self.text_log.see(END)
                except Exception:
                    pass

                # 若机器人处于错误模式（编号 9），则尝试拉取并展示错误详细信息
                if a["RobotMode"] == 9:
                    self.display_error_info()

    def display_error_info(self):
        """展示错误/报警信息。
        
        优先:
            Dashboard.GetError("en") 返回结构化数据。
        回退:
            解析 Dashboard.GetErrorID() 中的 JSON 片段。
        输出:
            写入 self.text_err 文本框。
        """
        # 先尝试使用 GetError 接口（推荐，返回结构化信息）
        try:
            # 调用 Dashboard.GetError，传入 "en" 表示英文，便于 UI 展示
            error_info = self.client_dash.GetError("en")  # Use English for UI display
            # 如果返回中包含键 "errMsg" 且其中有错误条目
            if error_info and "errMsg" in error_info and error_info["errMsg"]:
                # 遍历每条错误，使用新的格式化函数输出到错误文本框
                for error in error_info["errMsg"]:
                    self.form_error_new(error)
                # 成功使用新接口后直接返回，不再执行回退逻辑
                return
        except Exception as e:
            # 捕获新接口调用异常，打印日志并继续执行回退方案
            print(f"GetError interface failed, using fallback method: {e}")
        
        # 回退到旧方法：GetErrorID 返回字符串，需要手动从花括号中提取 JSON 片段再解析
        try:
            # 取返回字符串中第一个 "{" 与 "}" 之间的内容
            error_list = self.client_dash.GetErrorID().split("{")[1].split("}")[0]
            # 将 JSON 片段解析成 Python 列表对象
            error_list = json.loads(error_list)
            # 打印解析后的错误列表（调试用）
            print("error_list:", error_list)
            # error_list[0] 通常为控制器错误 ID 列表，若存在则遍历并格式化输出
            if error_list[0]:
                for i in error_list[0]:
                    self.form_error(i, self.alarm_controller_dict,
                                    "Controller Error")

            # 从第 1 项开始通常为各个伺服轴的错误列表，逐轴检查
            for m in range(1, len(error_list)):
                if error_list[m]:
                    # 遍历该轴错误列表的索引，逐条格式化输出
                    for n in range(len(error_list[m])):
                        self.form_error(n, self.alarm_servo_dict, "Servo Error")
        except Exception as e:
            # 如果两种方式都失败，则打印异常信息
            print(f"Both error retrieval methods failed: {e}")

    def form_error_new(self, error_data):
        """格式化并输出新接口 GetError 的单条错误信息。
        
        参数:
            error_data (dict): 包含时间、id、level、description、solution 等字段。
        返回:
            None
        副作用:
            向 self.text_err 追加可读文本。
        """
        try:
            # 拼接时间戳、ID、类别、等级、描述、解决方案等文本（安全使用 dict.get 提供默认值）
            error_info = f"Time Stamp:{error_data.get('date', 'N/A')} {error_data.get('time', 'N/A')}\n"
            error_info += f"ID:{error_data.get('id', 'N/A')}\n"
            error_info += f"Type:{error_data.get('mode', 'N/A')}\n"
            error_info += f"Level:{error_data.get('level', 'N/A')}\n"
            error_info += f"Description:{error_data.get('description', 'N/A')}\n"
            error_info += f"Solution:{error_data.get('solution', 'N/A')}\n\n"
            
            # 将格式化好的文本插入到错误信息文本框末尾
            self.text_err.insert(END, error_info)
        except Exception as e:
            # 捕获格式化过程中的异常并打印日志
            print(f"Error formatting new error data: {e}")
    
    def form_error(self, index, alarm_dict: dict, type_text):
        """格式化并输出旧接口映射到的错误信息。
        
        参数:
            index (int): 报警 ID。
            alarm_dict (dict): {id: info} 映射字典。
            type_text (str): 错误类别文本（如 Controller Error/Servo Error）。
        返回:
            None
        副作用:
            向 self.text_err 追加可读文本。
        """
        # 仅当报警 id 存在于字典中时才进行输出
        if index in alarm_dict.keys():
            # 生成当前本地时间作为时间戳
            date = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            # 逐行拼接：时间戳、ID、类型、等级、解决方案（英文）
            error_info = f"Time Stamp:{date}\n"
            error_info = error_info + f"ID:{index}\n"
            error_info = error_info + \
                f"Type:{type_text}\nLevel:{alarm_dict[index]['level']}\n" + \
                f"Solution:{alarm_dict[index]['en']['solution']}\n"

            # 将错误信息插入到错误文本框末尾
            self.text_err.insert(END, error_info)

    def clear_error_info(self):
        """清空错误信息显示区域（self.text_err）。"""
        # 从文本第 1 行第 0 列开始到“end”全部删除
        self.text_err.delete("1.0", "end")

    def set_feed_joint(self, label, value):
        """将 6 维数组值写入界面对应的 6 个标签。
        
        参数:
            label (list[list[str]]): LABEL_JOINT 或 LABEL_COORD。
            value (np.ndarray): 形如 [[v0..v5]] 的数值数组。
        返回:
            None
        """
        # 使用 numpy.around 对数组取 4 位小数，便于美观显示
        array_value = np.around(value, decimals=4)
        # label 参数是 LABEL_JOINT 或 LABEL_COORD，label[1] 为中间一排标签文本（如 "J1:", "X:" 等）
        # 依次把相应的数值写入之前保存的 label_feed_dict 的各个 Label 的 text 属性
        self.label_feed_dict[label[1][0]]["text"] = array_value[0][0]
        self.label_feed_dict[label[1][1]]["text"] = array_value[0][1]
        self.label_feed_dict[label[1][2]]["text"] = array_value[0][2]
        self.label_feed_dict[label[1][3]]["text"] = array_value[0][3]
        self.label_feed_dict[label[1][4]]["text"] = array_value[0][4]
        self.label_feed_dict[label[1][5]]["text"] = array_value[0][5]
