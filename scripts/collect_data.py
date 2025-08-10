#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
竞赛数据收集工具
Competition Data Collection Tool
"""

import os
import cv2
import json
import time
import argparse
from datetime import datetime
from pathlib import Path
import numpy as np

class CompetitionDataCollector:
    def __init__(self, output_dir="training_data", camera_id=0):
        """初始化数据收集器"""
        self.output_dir = Path(output_dir)
        self.camera_id = camera_id
        self.cap = None
        
        # 创建目录结构
        self.create_directory_structure()
        
        # 类别定义
        self.classes = {
            '0': 'square_frame',
            '1': 'circle_frame', 
            '2': 'red_flag',
            '3': 'blue_flag',
            '4': 'green_flag'
        }
        
        # 收集统计
        self.collection_stats = {cls: 0 for cls in self.classes.values()}
        
        # 当前标注状态
        self.current_class = '0'
        self.current_bbox = None
        self.drawing = False
        self.start_point = None
        
    def create_directory_structure(self):
        """创建数据目录结构"""
        splits = ['train', 'val', 'test']
        subdirs = ['images', 'labels']
        
        for split in splits:
            for subdir in subdirs:
                (self.output_dir / split / subdir).mkdir(parents=True, exist_ok=True)
        
        print(f"数据目录已创建: {self.output_dir}")
    
    def initialize_camera(self):
        """初始化相机"""
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开相机 {self.camera_id}")
        
        # 设置相机参数
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        print("相机初始化成功")
    
    def mouse_callback(self, event, x, y, flags, param):
        """鼠标回调函数 - 用于标注边界框"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.start_point = (x, y)
            
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing:
                # 实时显示正在绘制的框
                pass
                
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            end_point = (x, y)
            
            # 计算边界框
            x1, y1 = self.start_point
            x2, y2 = end_point
            
            # 确保坐标顺序正确
            x1, x2 = min(x1, x2), max(x1, x2)
            y1, y2 = min(y1, y2), max(y1, y2)
            
            # 检查框的大小
            if x2 - x1 > 20 and y2 - y1 > 20:
                self.current_bbox = (x1, y1, x2, y2)
                print(f"标注框: ({x1}, {y1}, {x2}, {y2}), 类别: {self.classes[self.current_class]}")
    
    def draw_interface(self, frame):
        """绘制用户界面"""
        height, width = frame.shape[:2]
        
        # 绘制当前类别信息
        class_name = self.classes[self.current_class]
        cv2.putText(frame, f"Current Class: {class_name} ({self.current_class})", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 绘制统计信息
        y_offset = 60
        for class_name, count in self.collection_stats.items():
            cv2.putText(frame, f"{class_name}: {count}", 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += 25
        
        # 绘制操作提示
        instructions = [
            "Keys: 0-4 (select class), Space (save), Q (quit)",
            "Mouse: Click and drag to draw bounding box",
            "ESC: Clear current box"
        ]
        
        y_start = height - 80
        for i, instruction in enumerate(instructions):
            cv2.putText(frame, instruction, (10, y_start + i * 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        # 绘制当前边界框
        if self.current_bbox:
            x1, y1, x2, y2 = self.current_bbox
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, self.classes[self.current_class], 
                       (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        return frame
    
    def save_annotation(self, frame):
        """保存标注数据"""
        if self.current_bbox is None:
            print("没有标注框，跳过保存")
            return False
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        
        # 决定保存到哪个分割 (80% train, 15% val, 5% test)
        rand_num = np.random.random()
        if rand_num < 0.8:
            split = 'train'
        elif rand_num < 0.95:
            split = 'val'
        else:
            split = 'test'
        
        # 保存图像
        image_filename = f"{self.classes[self.current_class]}_{timestamp}.jpg"
        image_path = self.output_dir / split / 'images' / image_filename
        cv2.imwrite(str(image_path), frame)
        
        # 保存标注 (YOLO格式)
        label_filename = f"{self.classes[self.current_class]}_{timestamp}.txt"
        label_path = self.output_dir / split / 'labels' / label_filename
        
        # 转换为YOLO格式 (相对坐标)
        height, width = frame.shape[:2]
        x1, y1, x2, y2 = self.current_bbox
        
        # 计算中心点和宽高 (相对坐标)
        x_center = ((x1 + x2) / 2) / width
        y_center = ((y1 + y2) / 2) / height
        bbox_width = (x2 - x1) / width
        bbox_height = (y2 - y1) / height
        
        # 写入标注文件
        with open(label_path, 'w') as f:
            f.write(f"{self.current_class} {x_center:.6f} {y_center:.6f} {bbox_width:.6f} {bbox_height:.6f}\n")
        
        # 更新统计
        class_name = self.classes[self.current_class]
        self.collection_stats[class_name] += 1
        
        print(f"已保存: {image_filename} -> {split}")
        
        # 清除当前标注
        self.current_bbox = None
        
        return True
    
    def run_collection(self):
        """运行数据收集"""
        self.initialize_camera()
        
        # 设置窗口和鼠标回调
        cv2.namedWindow('Data Collection', cv2.WINDOW_RESIZABLE)
        cv2.setMouseCallback('Data Collection', self.mouse_callback)
        
        print("数据收集已启动...")
        print("操作说明:")
        print("- 数字键 0-4: 选择类别")
        print("- 鼠标拖拽: 绘制边界框")
        print("- 空格键: 保存当前标注")
        print("- ESC键: 清除当前标注框")
        print("- Q键: 退出程序")
        
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("无法读取相机画面")
                    break
                
                # 绘制界面
                display_frame = self.draw_interface(frame.copy())
                
                # 如果正在绘制，显示临时框
                if self.drawing and self.start_point:
                    mouse_pos = cv2.getMousePos('Data Collection')
                    if mouse_pos != (-1, -1):
                        cv2.rectangle(display_frame, self.start_point, mouse_pos, (255, 0, 0), 1)
                
                cv2.imshow('Data Collection', display_frame)
                
                # 处理按键
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    break
                elif key in [ord('0'), ord('1'), ord('2'), ord('3'), ord('4')]:
                    self.current_class = chr(key)
                    print(f"切换到类别: {self.classes[self.current_class]}")
                elif key == ord(' '):  # 空格键保存
                    self.save_annotation(frame)
                elif key == 27:  # ESC键清除
                    self.current_bbox = None
                    print("清除当前标注框")
                    
        except KeyboardInterrupt:
            print("\n收集被中断")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        
        # 保存收集统计
        stats_file = self.output_dir / 'collection_stats.json'
        with open(stats_file, 'w') as f:
            json.dump({
                'collection_stats': self.collection_stats,
                'total_samples': sum(self.collection_stats.values()),
                'collection_date': datetime.now().isoformat(),
                'classes': self.classes
            }, f, indent=2)
        
        print(f"\n收集统计已保存到: {stats_file}")
        print("总计收集:")
        for class_name, count in self.collection_stats.items():
            print(f"  {class_name}: {count} 张")
        print(f"  总计: {sum(self.collection_stats.values())} 张")

def main():
    parser = argparse.ArgumentParser(description='竞赛数据收集工具')
    parser.add_argument('--output-dir', type=str, default='training_data',
                       help='输出目录')
    parser.add_argument('--camera-id', type=int, default=0,
                       help='相机设备ID')
    
    args = parser.parse_args()
    
    collector = CompetitionDataCollector(args.output_dir, args.camera_id)
    collector.run_collection()

if __name__ == "__main__":
    main()