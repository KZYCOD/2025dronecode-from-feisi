
# import required libraries
# pip3 install pymavlink pyserial


import cv2
import numpy as np
import time
import VisionCaptureApi
import math
import ReqCopterSim

# import sys
# # 自定义跟踪函数
# def trace_calls(frame, event, arg):
#     if event == 'call':
#         print(f"调用函数: {frame.f_code.co_name}，文件: {frame.f_code.co_filename}，行号: {frame.f_lineno}")
#     return trace_calls

# # 设置全局跟踪
# sys.settrace(trace_calls)
            
            
req = ReqCopterSim.ReqCopterSim()
# 获取ID和IP列表
# IPList=req.getSimIpList()
# print(IPList)

CopterID=1 

# 获取到指定CopterID的CopterSim所在电脑的IP
TargetIP = req.getSimIpID(CopterID)

# 请求目标CopterSim将数据返回到本电脑
# 通过本接口，可以不用再去bat脚本里面填写IP地址了
req.sendReSimIP(CopterID)

# 通过本接口，可以强制修改CopterSim的mavlink_version，这里只有要时，才发送
# new_UDP_mode=6
# req.sendReSimUdpMode(CopterID,new_UDP_mode)


VisionCaptureApi.isEnableRosTrans = True
vis = VisionCaptureApi.VisionCaptureApi(ip=TargetIP)



# VisionCaptureApi 中的配置函数
vis.jsonLoad(1)  # 加载Config.json中的传感器配置文件
isSuss = vis.sendReqToUE4(
    0, TargetIP
)

vis.startImgCap()  # 开启取图循环，执行本语句之后，已经可以通过vis.Img[i]读取到图片了

vis.RemotSendIP = req.hostIp

vis.sendImuReqCopterSim(IP=TargetIP)
