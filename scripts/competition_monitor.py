#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
竞赛系统监控工具
Competition System Monitor
"""

import rospy
import psutil
import time
import json
import threading
from datetime import datetime
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState, Image
import matplotlib.pyplot as plt
import numpy as np

class CompetitionMonitor:
    def __init__(self):
        """初始化监控系统"""
        rospy.init_node('competition_monitor', anonymous=True)
        
        # 监控数据存储
        self.data = {
            'timestamp': [],
            'system_cpu': [],
            'system_memory': [],
            'battery_voltage': [],
            'battery_percentage': [],
            'position': [],
            'competition_state': [],
            'detection_count': [],
            'mavros_connected': [],
            'camera_fps': []
        }
        
        # 状态变量
        self.last_image_time = time.time()
        self.image_count = 0
        self.detection_count = 0
        self.current_position = None
        self.current_competition_state = "UNKNOWN"
        self.mavros_connected = False
        self.battery_info = None
        
        # 监控配置
        self.monitor_interval = 1.0  # 监控间隔(秒)
        self.running = True
        
        # 设置ROS订阅者
        self.setup_subscribers()
        
        # 启动监控线程
        self.monitor_thread = threading.Thread(target=self.monitor_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
        print("竞赛系统监控已启动")
    
    def setup_subscribers(self):
        """设置ROS订阅者"""
        # 竞赛状态监控
        rospy.Subscriber('/competition/state', String, self.competition_state_callback)
        rospy.Subscriber('/competition/obstacle_complete', Bool, self.obstacle_complete_callback)
        rospy.Subscriber('/competition/current_obstacle', Int32, self.current_obstacle_callback)
        
        # 飞行状态监控
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.position_callback)
        rospy.Subscriber('/mavros/state', State, self.mavros_state_callback)
        rospy.Subscriber('/mavros/battery', BatteryState, self.battery_callback)
        
        # 视觉系统监控
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        rospy.Subscriber('/object_detection/results', String, self.detection_callback)
    
    def competition_state_callback(self, msg):
        """竞赛状态回调"""
        self.current_competition_state = msg.data
    
    def obstacle_complete_callback(self, msg):
        """障碍完成回调"""
        if msg.data:
            print(f"[{datetime.now().strftime('%H:%M:%S')}] 障碍完成")
    
    def current_obstacle_callback(self, msg):
        """当前障碍回调"""
        print(f"[{datetime.now().strftime('%H:%M:%S')}] 当前障碍: obs{msg.data}")
    
    def position_callback(self, msg):
        """位置回调"""
        self.current_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
    
    def mavros_state_callback(self, msg):
        """MAVROS状态回调"""
        self.mavros_connected = msg.connected
        if not msg.connected:
            print(f"[{datetime.now().strftime('%H:%M:%S')}] 警告: MAVROS连接丢失")
    
    def battery_callback(self, msg):
        """电池状态回调"""
        self.battery_info = {
            'voltage': msg.voltage,
            'percentage': msg.percentage * 100  # 转换为百分比
        }
        
        # 低电量警告
        if msg.percentage < 0.3:  # 30%以下
            print(f"[{datetime.now().strftime('%H:%M:%S')}] 警告: 电池电量低 ({msg.percentage*100:.1f}%)")
    
    def image_callback(self, msg):
        """图像回调 - 计算相机帧率"""
        current_time = time.time()
        self.image_count += 1
        
        # 每5秒计算一次帧率
        if current_time - self.last_image_time > 5.0:
            fps = self.image_count / (current_time - self.last_image_time)
            self.camera_fps = fps
            self.image_count = 0
            self.last_image_time = current_time
    
    def detection_callback(self, msg):
        """检测结果回调"""
        self.detection_count += 1
    
    def get_system_stats(self):
        """获取系统统计信息"""
        # CPU使用率
        cpu_percent = psutil.cpu_percent(interval=None)
        
        # 内存使用率
        memory = psutil.virtual_memory()
        memory_percent = memory.percent
        
        # GPU使用率 (如果有NVIDIA GPU)
        gpu_percent = 0
        try:
            import pynvml
            pynvml.nvmlInit()
            handle = pynvml.nvmlDeviceGetHandleByIndex(0)
            gpu_util = pynvml.nvmlDeviceGetUtilizationRates(handle)
            gpu_percent = gpu_util.gpu
        except:
            pass
        
        return {
            'cpu': cpu_percent,
            'memory': memory_percent,
            'gpu': gpu_percent
        }
    
    def monitor_loop(self):
        """主监控循环"""
        while self.running and not rospy.is_shutdown():
            try:
                # 获取当前时间戳
                timestamp = time.time()
                
                # 获取系统统计
                system_stats = self.get_system_stats()
                
                # 记录数据
                self.data['timestamp'].append(timestamp)
                self.data['system_cpu'].append(system_stats['cpu'])
                self.data['system_memory'].append(system_stats['memory'])
                
                if self.battery_info:
                    self.data['battery_voltage'].append(self.battery_info['voltage'])
                    self.data['battery_percentage'].append(self.battery_info['percentage'])
                else:
                    self.data['battery_voltage'].append(0)
                    self.data['battery_percentage'].append(0)
                
                if self.current_position:
                    self.data['position'].append(self.current_position.copy())
                else:
                    self.data['position'].append([0, 0, 0])
                
                self.data['competition_state'].append(self.current_competition_state)
                self.data['detection_count'].append(self.detection_count)
                self.data['mavros_connected'].append(self.mavros_connected)
                self.data['camera_fps'].append(getattr(self, 'camera_fps', 0))
                
                # 保持数据长度合理 (最近10分钟)
                max_length = int(600 / self.monitor_interval)  # 10分钟
                if len(self.data['timestamp']) > max_length:
                    for key in self.data:
                        self.data[key] = self.data[key][-max_length:]
                
                # 检查异常情况
                self.check_anomalies(system_stats)
                
            except Exception as e:
                print(f"监控循环错误: {e}")
            
            time.sleep(self.monitor_interval)
    
    def check_anomalies(self, system_stats):
        """检查异常情况"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        
        # CPU过高
        if system_stats['cpu'] > 90:
            print(f"[{timestamp}] 警告: CPU使用率过高 ({system_stats['cpu']:.1f}%)")
        
        # 内存过高
        if system_stats['memory'] > 90:
            print(f"[{timestamp}] 警告: 内存使用率过高 ({system_stats['memory']:.1f}%)")
        
        # 相机帧率过低
        if hasattr(self, 'camera_fps') and self.camera_fps > 0 and self.camera_fps < 15:
            print(f"[{timestamp}] 警告: 相机帧率过低 ({self.camera_fps:.1f} fps)")
        
        # MAVROS连接状态
        if not self.mavros_connected:
            print(f"[{timestamp}] 错误: MAVROS未连接")
    
    def generate_report(self):
        """生成监控报告"""
        if not self.data['timestamp']:
            print("没有监控数据")
            return
        
        # 创建图表
        fig, axes = plt.subplots(3, 2, figsize=(15, 12))
        fig.suptitle('竞赛系统监控报告', fontsize=16)
        
        # 时间轴 (转换为相对时间)
        start_time = self.data['timestamp'][0]
        time_axis = [(t - start_time) / 60 for t in self.data['timestamp']]  # 分钟
        
        # CPU和内存使用率
        axes[0, 0].plot(time_axis, self.data['system_cpu'], label='CPU %', color='red')
        axes[0, 0].plot(time_axis, self.data['system_memory'], label='Memory %', color='blue')
        axes[0, 0].set_title('系统资源使用率')
        axes[0, 0].set_ylabel('使用率 (%)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        # 电池状态
        axes[0, 1].plot(time_axis, self.data['battery_percentage'], label='电量 %', color='green')
        battery_voltage = [v for v in self.data['battery_voltage'] if v > 0]
        if battery_voltage:
            voltage_time = time_axis[:len(battery_voltage)]
            axes_twin = axes[0, 1].twinx()
            axes_twin.plot(voltage_time, battery_voltage, label='电压 V', color='orange')
            axes_twin.set_ylabel('电压 (V)')
        axes[0, 1].set_title('电池状态')
        axes[0, 1].set_ylabel('电量 (%)')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
        
        # 位置轨迹 (XY平面)
        positions = np.array(self.data['position'])
        if len(positions) > 0:
            axes[1, 0].plot(positions[:, 0], positions[:, 1], 'b-', alpha=0.7)
            axes[1, 0].scatter(positions[0, 0], positions[0, 1], color='green', s=100, label='起点')
            axes[1, 0].scatter(positions[-1, 0], positions[-1, 1], color='red', s=100, label='终点')
            axes[1, 0].set_title('飞行轨迹 (XY平面)')
            axes[1, 0].set_xlabel('X (m)')
            axes[1, 0].set_ylabel('Y (m)')
            axes[1, 0].legend()
            axes[1, 0].grid(True)
            axes[1, 0].axis('equal')
        
        # 高度变化
        if len(positions) > 0:
            axes[1, 1].plot(time_axis, positions[:, 2], 'purple')
            axes[1, 1].set_title('高度变化')
            axes[1, 1].set_ylabel('高度 (m)')
            axes[1, 1].grid(True)
        
        # 检测统计
        detection_counts = np.array(self.data['detection_count'])
        if len(detection_counts) > 0:
            detection_rate = np.diff(detection_counts, prepend=0)  # 每秒检测数
            axes[2, 0].plot(time_axis, detection_rate, 'orange')
            axes[2, 0].set_title('目标检测率')
            axes[2, 0].set_ylabel('检测数/秒')
            axes[2, 0].grid(True)
        
        # 相机帧率
        axes[2, 1].plot(time_axis, self.data['camera_fps'], 'cyan')
        axes[2, 1].set_title('相机帧率')
        axes[2, 1].set_ylabel('FPS')
        axes[2, 1].set_xlabel('时间 (分钟)')
        axes[2, 1].grid(True)
        
        plt.tight_layout()
        
        # 保存报告
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        report_filename = f'competition_monitor_report_{timestamp}.png'
        plt.savefig(report_filename, dpi=300, bbox_inches='tight')
        print(f"监控报告已保存: {report_filename}")
        
        # 保存数据
        data_filename = f'competition_monitor_data_{timestamp}.json'
        with open(data_filename, 'w') as f:
            # 转换数据为可序列化格式
            export_data = {}
            for key, values in self.data.items():
                if key == 'position':
                    export_data[key] = [list(pos) for pos in values]
                else:
                    export_data[key] = list(values)
            json.dump(export_data, f, indent=2)
        
        print(f"监控数据已保存: {data_filename}")
        
        plt.show()
    
    def print_status(self):
        """打印当前状态"""
        print("\n" + "="*50)
        print("竞赛系统实时状态")
        print("="*50)
        
        # 系统资源
        system_stats = self.get_system_stats()
        print(f"CPU使用率: {system_stats['cpu']:.1f}%")
        print(f"内存使用率: {system_stats['memory']:.1f}%")
        
        # 电池状态
        if self.battery_info:
            print(f"电池电量: {self.battery_info['percentage']:.1f}%")
            print(f"电池电压: {self.battery_info['voltage']:.2f}V")
        else:
            print("电池状态: 未知")
        
        # 位置信息
        if self.current_position:
            print(f"当前位置: ({self.current_position[0]:.2f}, {self.current_position[1]:.2f}, {self.current_position[2]:.2f})")
        else:
            print("位置信息: 未知")
        
        # 竞赛状态
        print(f"竞赛状态: {self.current_competition_state}")
        print(f"MAVROS连接: {'正常' if self.mavros_connected else '断开'}")
        
        # 相机状态
        if hasattr(self, 'camera_fps'):
            print(f"相机帧率: {self.camera_fps:.1f} fps")
        
        print(f"检测计数: {self.detection_count}")
        print("="*50)
    
    def stop(self):
        """停止监控"""
        self.running = False
        print("监控已停止")

def main():
    try:
        monitor = CompetitionMonitor()
        
        print("竞赛监控系统已启动")
        print("按键说明:")
        print("  Enter: 显示当前状态")
        print("  'r': 生成监控报告")
        print("  'q': 退出")
        
        while not rospy.is_shutdown():
            try:
                user_input = input().strip().lower()
                
                if user_input == 'q':
                    break
                elif user_input == 'r':
                    monitor.generate_report()
                elif user_input == '':
                    monitor.print_status()
                else:
                    print("未知命令")
                    
            except KeyboardInterrupt:
                break
        
        monitor.stop()
        
    except rospy.ROSInterruptException:
        print("ROS中断")
    except Exception as e:
        print(f"监控系统错误: {e}")

if __name__ == "__main__":
    main()