from dobot_api import DobotApiFeedBack,DobotApiDashboard
import threading
from time import sleep
import re

class DobotDemo:
    def __init__(self, ip):
        self.ip = ip    #ip是机器人的网络地址
        self.dashboardPort = 29999  # 机器人的"控制端口"：发指令用的端口号（固定值，Dobot机器人约定的）
        self.feedPortFour = 30004   # 机器人的"状态反馈端口"：接收状态用的端口号（固定值）
        self.dashboard = None   # 先定义一个变量，后面用来存"控制指令对象"（暂时为空）
        self.feedInfo = []  # 定义一个列表，暂时用来存状态信息（后面实际用feedData）
        self.__globalLockValue = threading.Lock()   # 创建一个"线程锁"：防止多个线程同时修改数据导致混乱（比如一个线程写状态，一个线程读状态，加锁保证安全）
        
        class item:
            def __init__(self): # 在DobotDemo类里面定义一个小类item，用来专门存机器人的实时状态数据
                self.robotMode = -1     # 机器人当前模式
                self.robotCurrentCommandID = 0  # 当前正在执行的指令ID
                self.MessageSize = -1   # 状态消息的长度
                self.DigitalInputs =-1  # 数字输入信号
                self.DigitalOutputs = -1    # 数字输出信号
                self.robotCurrentCommandID = -1
                # 自定义添加所需反馈数据

        self.feedData = item()  # 创建item类的实例,存储机器人状态

    def start(self):
        # 启动机器人并使能
        self.dashboard = DobotApiDashboard(self.ip, self.dashboardPort) #创建控制指令对象：连接机器人的控制端口（29999）
        self.feedFour = DobotApiFeedBack(self.ip, self.feedPortFour)# 创建状态反馈对象：连接机器人的状态端口（30004）
        if self.parseResultId(self.dashboard.EnableRobot())[0] != 0:
            print("使能失败: 检查29999端口是否被占用")
            return
        print("使能成功")

        # 启动状态反馈线程
        feed_thread = threading.Thread(
            target=self.GetFeed)  # 创建一个线程，任务是执行GetFeed方法（持续获取机器人状态）
        feed_thread.daemon = True   # 设置为"守护线程"：主线程结束时，这个线程也自动结束（避免程序退出后线程还在跑）
        feed_thread.start() # 启动这个线程，开始执行GetFeed方法

        # 定义两个目标点
        point_a = [146.3759,-283.4321,332.3956,177.7879,-1.8540,147.5821]   #每个元素的含义是：[X 坐标，Y 坐标，Z 坐标，Rx 旋转角，Ry 旋转角，Rz 旋转角]（单位一般是毫米和度）
        point_b = [146.3759,-283.4321,432.3956,177.7879,-1.8540,147.5821]

        # 走点循环
        while True:
            print("DI:", self.feedData.DigitalInputs,"2DI:", bin(self.feedData.DigitalInputs),"--16:",hex(self.feedData.DigitalInputs)) # 打印数字输入信号（十进制、二进制、十六进制）
            print("DO:", self.feedData.DigitalOutputs,"2DO:" ,bin(self.feedData.DigitalOutputs),"--16:",hex(self.feedData.DigitalOutputs))  # 打印数字输出信号（同上）
            print("robomode",self.feedData.robotMode)
            sleep(2)

    def GetFeed(self):
        # 获取机器人状态
        while True:
            feedInfo = self.feedFour.feedBackData() # 调用feedFour的方法，获取机器人的实时状态数据（返回一个字典）
            with self.__globalLockValue:    # 用线程锁保护下面的代码：确保同一时间只有一个线程读写feedData
                if feedInfo is not None:   
                    if hex((feedInfo['TestValue'][0])) == '0x123456789abcdef':   # 验证数据是否有效
                        # 基础字段
                        self.feedData.MessageSize = feedInfo['len'][0]  # 把状态消息的长度存到feedData里
                        self.feedData.robotMode = feedInfo['RobotMode'][0] # 把机器人模式存到feedData里
                        self.feedData.DigitalInputs = feedInfo['DigitalInputs'][0] # 把数字输入存到feedData里
                        self.feedData.DigitalOutputs = feedInfo['DigitalOutputs'][0] # 把数字输出存到feedData里
                        self.feedData.robotCurrentCommandID = feedInfo['CurrentCommandId'][0]  # 把当前指令ID存到feedData里
                        # 自定义添加所需反馈数据
                        '''
                        self.feedData.DigitalOutputs = int(feedInfo['DigitalOutputs'][0])
                        self.feedData.RobotMode = int(feedInfo['RobotMode'][0])
                        self.feedData.TimeStamp = int(feedInfo['TimeStamp'][0])
                        '''

    def RunPoint(self, point_list): # 接收一个点坐标列表，控制机器人移动到该点
        # 走点指令
        recvmovemess = self.dashboard.MovJ(*point_list, 0)  # 发送"关节运动"指令（MovJ是快速移动，是"关节空间运动"，路径不固定）
        print("MovJ:", recvmovemess)    #recvmovemess 是机器人返回的消息
        print(self.parseResultId(recvmovemess)) # 打印解析后的消息结果
        currentCommandID = self.parseResultId(recvmovemess)[1]  # 从解析结果中取第二个值，作为当前指令的ID
        print("指令 ID:", currentCommandID)
        #sleep(0.02)
        while True:  #完成判断循环

            print(self.feedData.robotMode)
            if self.feedData.robotMode == 5 and self.feedData.robotCurrentCommandID == currentCommandID:
                print("运动结束")
                break
            sleep(0.1)

    def parseResultId(self, valueRecv):
        # 解析返回值，确保机器人在 TCP 控制模式
        if "Not Tcp" in valueRecv:
            print("Control Mode Is Not Tcp")
            return [1]
        return [int(num) for num in re.findall(r'-?\d+', valueRecv)] or [2]

    def __del__(self):  #析构方法，当类的实例被删除时自动调用
        del self.dashboard  # 删除控制指令对象，断开与机器人控制端口的连接
        del self.feedFour   # 删除状态反馈对象，断开与机器人状态端口的连接
