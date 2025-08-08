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
red_channel_th = 60 #红色通道的阈值

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
        red_mask = red_channel > threshold
        red_pixel_coords = np.column_stack(np.where(red_mask))
        # 计算红色像素的中心位置（相对于ROI）
        if len(red_pixel_coords) > 0:
            center_roi = np.mean(red_pixel_coords, axis=0).astype(int)
            # 将中心位置转换为原始图像坐标系
            center_original = (center_roi[1] + left[0], center_roi[0] + left[1])
            # print(f"红色像素的中心位置（原始图像坐标系）: {center_original}")
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
            print("没有找到红色像素")
            center_original = None
        if center_original is not None:
            cv2.circle(img, center_original, 5, (0, 255, 0), -1)  # 在中心位置画一个绿色圆点   
        return center_original
                
    def run(self):

        if(self.img_msg is None):
            return
        self.color_lock.acquire()

        self.color_img = self.img_bridge.imgmsg_to_cv2(self.img_msg, self.img_msg.encoding)
        if self.is_sim:
            # For simulation, use BGR to RGB conversion
            img = cv2.cvtColor(self.color_img, cv2.COLOR_BGR2RGB)
        else:
            # For real world camera, use image directly without color conversion
            img = self.color_img.copy()
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
            # print("det ", det)
            
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
                    # Use different thresholds for simulation vs real world
                    threshold = 60 if self.is_sim else 100  # Higher threshold for real world
                    cnt =  self.roi_track(img_yolo,xy[0:2],xy[2:4],threshold) 
                    if(cnt == None):
                        print("当前逻辑出错")
                        return
                    else:
                        obj.center_x = cnt[0]
                        obj.center_y = cnt[1]
                else:
                    obj.center_x = int((xy[0] + xy[2])/2)
                    obj.center_y = int((xy[1] + xy[3])/2)
                obj.score = det[i,4]
                obj.header = header
                objs.header = header
                objs.source_id = self.sensor_id
                objs.objects.append(obj)
            self.ret_pub.publish(objs)
        if self.show_img:
            if len(det) != 0:
                # 把最后选中的框用黑色框画出来
                cv2.rectangle(img_yolo, tuple(xy[:2]), tuple(xy[2:4]), (0,0,0), thickness=3, lineType=cv2.LINE_AA)
            cv2.imshow('det', img_yolo)
    

        # self.color_done = False
        # self.color_lock.release()
        


if __name__ == "__main__":
    rospy.init_node('det_node')
    topic1 = "/camera/color/image_raw" #前视相机的图像
    is_sim = False #仿真与真机的颜色通道值不同 - Set to False for real world operation
    img_proc = Depth_Estimate(img1_topic = topic1,sim = is_sim)
    while not rospy.is_shutdown():
        img_proc.run()
        if cv2.waitKey(10) == ord('q'):
            break
        time.sleep(0.03)
    # rospy.spin()

