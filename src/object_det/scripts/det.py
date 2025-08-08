#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import os
import sys
import cv2
import numpy as np
import rospy
import time
import threading
from rospy import Time
from sensor_msgs.msg import Image
from common_msgs.msg import Objects
from common_msgs.msg import Obj
from common_msgs.msg import DetEnable
from cv_bridge import CvBridge
from ObjectDetect import Yolo_Detect
from functools import partial


script_path = os.path.abspath(__file__)
script_dir = os.path.dirname(script_path)
sys.path.append(script_dir)
class_name =["frame","balloon"]
# -----------------------------参数--------------------------------
# yolo模型路径
Model_path = script_dir + '/Model/frame_balloon.pt'
# 红色通道的阈值 - 仿真和真实环境使用不同的阈值
red_channel_th_sim = 60    # 仿真环境的红色通道阈值
red_channel_th_real = 120  # 真实环境的红色通道阈值（真实环境光照更强，需要更高阈值）

# ----------------------------------------------------------------

class Depth_Estimate:
    def __init__(self,img1_topic ,sim):
        self.yolo_detector = Yolo_Detect(Model_path)
        self.img_bridge = CvBridge()

        self.color_done = False
        # 是否显示中间结果图片
        self.show_img = True
        self.color_img1 = None
        self.color_img2 = None
        self.img_msg = None
        self.header = None
        self.task = None
        self.is_sim = sim
        self.sensor_id = 0
        
        # 根据是否为仿真环境设置红色阈值
        self.red_threshold = red_channel_th_sim if self.is_sim else red_channel_th_real
        rospy.loginfo(f"Detection mode: {'Simulation' if self.is_sim else 'Real'}, Red threshold: {self.red_threshold}")
        

        self.color_lock = threading.Lock()
        # self.img1_sub = rospy.Subscriber(img1_topic, Image, partial(self.img_cb, idx=1))
        self.img1_sub = rospy.Subscriber(img1_topic, Image,self.img_cb)
        self.task_sub = rospy.Subscriber("/detect_ctrl",DetEnable,self.ctrl_cb)
        self.ret_pub = rospy.Publisher("/objects",Objects,queue_size=10)

    def ctrl_cb(self,msg):
        self.task = msg
    def img_cb(self, msg):
        self.img_msg = msg
        self.color_lock.acquire()
        self.img_msg = msg
        self.color_lock.release()
        
    def roi_track(self,img,left:tuple,right:tuple,threshold =200):
        roi = img[left[1] :right[1],left[0]:right[0]] #img[行，列]
        red_channel = roi[:,:,2]
        
        # 如果使用默认阈值没有找到红色像素，尝试动态调整阈值
        red_mask = red_channel > threshold
        red_pixel_coords = np.column_stack(np.where(red_mask))
        
        # 如果没有找到红色像素，尝试降低阈值（仅在真实环境中）
        if len(red_pixel_coords) == 0 and not self.is_sim:
            # 计算ROI中红色通道的统计信息
            red_mean = np.mean(red_channel)
            red_std = np.std(red_channel)
            red_max = np.max(red_channel)
            
            # 动态调整阈值：使用均值+标准差或最大值的80%，取较小值
            adaptive_threshold = min(red_mean + red_std, red_max * 0.8)
            if adaptive_threshold < threshold:
                threshold = adaptive_threshold
                red_mask = red_channel > threshold
                red_pixel_coords = np.column_stack(np.where(red_mask))
                rospy.loginfo(f"动态调整红色阈值为: {threshold:.1f} (均值: {red_mean:.1f}, 标准差: {red_std:.1f})")
        
        # 调试信息：输出红色像素点数量
        red_pixel_count = len(red_pixel_coords)
        rospy.logdebug(f"ROI size: {roi.shape}, Red threshold: {threshold}, Red pixels found: {red_pixel_count}")
        
        # 计算红色像素的中心位置（相对于ROI）
        if len(red_pixel_coords) > 0:
            center_roi = np.mean(red_pixel_coords, axis=0).astype(int)
            # 将中心位置转换为原始图像坐标系
            center_original = (center_roi[1] + left[0], center_roi[0] + left[1])
            rospy.logdebug(f"红色像素的中心位置（原始图像坐标系）: {center_original}")
            red_pixels_visualization = np.zeros_like(roi)
            red_pixels_visualization[red_mask] = [0, 0, 255]  # 将红色像素点标记为红色

            # 将红色像素点可视化结果覆盖到原始图像的ROI区域
            img[left[1] :right[1], left[0]:right[0]][red_mask] = [0, 0, 255]

            # 绘制感兴趣区域（ROI）的矩形框
            # cv2.rectangle(img, (left), right, (0, 255, 0), 2)
            self.show_img = True
            #cv2.imshow("segment",img)
            #cv2.waitKey(1)
            
        else:
            red_mean = np.mean(red_channel)
            rospy.logwarn(f"没有找到红色像素，阈值: {threshold:.1f}, ROI红色通道均值: {red_mean:.1f}")
            center_original = None
        if center_original is not None:
            cv2.circle(img, center_original, 5, (0, 255, 0), -1)  # 在中心位置画一个绿色圆点   
        return center_original
                
    def run(self):

        if(self.img_msg is None):
            return
        self.color_lock.acquire()

        self.color_img = self.img_bridge.imgmsg_to_cv2(self.img_msg, self.img_msg.encoding)
        img = cv2.cvtColor(self.color_img, cv2.COLOR_BGR2RGB)
#        cv2.imshow("img",img);
#        cv2.waitKey(1);
        #保证不用的数据不占用资源
        
        header = self.img_msg.header
        #self.color_img = np.frombuffer(self.img_msg.data, dtype=np.uint8).reshape(self.img_msg.height, self.img_msg.width, -1)
        self.color_lock.release()
        # img = self.color_img
        img_yolo, det, dt = self.yolo_detector(img)
        # if(self.task == None or not self.task.is_open): #如果不打开检测，关闭检测
        #     print("wait detect cmd!")
        #     return
        if len(det) != 0:
            # 根据置信度和面积取最可靠目标
            # scores = np.sqrt((det[:, 2] - det[:, 0]) * (det[:, 3] - det[:, 1])) * (det[:, 4]**2)
            # frame_index = np.argmax(scores)
            # xy = det[frame_index, :4].astype(int)
            # img_color_c = self.color_img[xy[1]:xy[3], xy[0]:xy[2], :]
            rospy.logdebug(f"检测到 {len(det)} 个目标")
            
            objs = Objects()
            for i in range(0,len(det)):
                xy = det[i, :4].astype(int)
                obj = Obj()
                obj.class_name=class_name[int(det[i,5])]
                obj.left_top_x = xy[0]
                obj.left_top_y = xy[1]
                obj.right_bottom_x = xy[2]
                obj.right_bottom_y = xy[3]
                if(obj.class_name == "balloon"): #对于气球我们需要提取红色像素点
                    rospy.loginfo(f"检测到气球，置信度: {det[i,4]:.2f}, 使用红色阈值: {self.red_threshold}")
                    cnt =  self.roi_track(img_yolo,xy[0:2],xy[2:4], self.red_threshold) 
                    if(cnt == None):
                        rospy.logwarn("当前逻辑出错或未找到红色像素点")
                        return
                    else:
                        obj.center_x = cnt[0]
                        obj.center_y = cnt[1]
                        rospy.loginfo(f"气球中心位置: ({obj.center_x}, {obj.center_y})")
                else:
                    obj.center_x = int((xy[0] + xy[2])/2)
                    obj.center_y = int((xy[1] + xy[3])/2)
                obj.score = det[i,4]
                obj.header = header
                objs.header = header
                objs.source_id = self.sensor_id
                objs.objects.append(obj)
            self.ret_pub.publish(objs)
            rospy.logdebug(f"发布了 {len(objs.objects)} 个检测目标")
        if self.show_img:
            if len(det) != 0:
                # 把最后选中的框用黑色框画出来
                cv2.rectangle(img_yolo, tuple(xy[:2]), tuple(xy[2:4]), (0,0,0), thickness=3, lineType=cv2.LINE_AA)
            cv2.imshow('det', img_yolo)
    

        # self.color_done = False
        # self.color_lock.release()
        


if __name__ == "__main__":
    rospy.init_node('det_node')
    
    # 自动检测是否为仿真环境：检查仿真特有的topic是否存在
    sim_topic = "/rflysim/sensor1/img_rgb"  # 仿真环境特有的topic
    real_topic = "/camera/color/image_raw"   # 真实环境使用的topic
    
    # 等待一段时间让topic列表更新
    rospy.sleep(2.0)
    
    # 获取当前所有topic
    published_topics = [topic[0] for topic in rospy.get_published_topics()]
    
    # 判断是否为仿真环境
    if sim_topic in published_topics:
        topic1 = sim_topic
        is_sim = True
        rospy.loginfo("检测到仿真环境，使用仿真摄像头topic")
    elif real_topic in published_topics:
        topic1 = real_topic  
        is_sim = False
        rospy.loginfo("检测到真实环境，使用真实摄像头topic")
    else:
        # 如果都没有，从参数服务器获取或使用默认值
        topic1 = rospy.get_param('~camera_topic', real_topic)
        is_sim = rospy.get_param('~is_sim', False)
        rospy.logwarn(f"未检测到预期的摄像头topic，使用参数配置: {topic1}, is_sim: {is_sim}")
    
    rospy.loginfo(f"使用摄像头topic: {topic1}, 仿真模式: {is_sim}")
    
    img_proc = Depth_Estimate(img1_topic = topic1, sim = is_sim)
    while not rospy.is_shutdown():
        img_proc.run()
        if cv2.waitKey(10) == ord('q'):
            break
        time.sleep(0.03)
    # rospy.spin()

