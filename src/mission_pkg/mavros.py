
# import required libraries
# pip3 install pymavlink pyserial

import cv2
import numpy as np
import time
import math
import ReqCopterSim
import sys
import RflyRosStart

req = ReqCopterSim.ReqCopterSim() # 获取局域网内所有CopterSim程序的电脑IP列表
TargetID = 1 # 初始飞机的ID号
TargetIP = req.getSimIpID(TargetID) # 获取CopterSim的1号程序所在电脑的IP，作为目标IP
# 注意：如果是本电脑运行的话，那TargetIP是127.0.0.1的本机地址；如果是远程访问，则是192打头的局域网地址。
# 因此本程序能同时在本机运行，也能在其他电脑运行。

if TargetIP=='':
    print('Failed to get IP of Copter #',TargetID)
    sys.exit(0)


print(TargetIP)

if not (RflyRosStart.isLinux and RflyRosStart.isRosOk):
    print('This demo can only run on with Ros')
    sys.exit(0)


req.sendReSimIP(TargetID) # 请求回传数据到本电脑
print('Send sendReSimIP')
req.sendReSimUdpMode(TargetID,2) #強制切換MAVLINK_FULL
print('sendReSimUdpMode')

ros = RflyRosStart.RflyRosStart(TargetID,TargetIP)
