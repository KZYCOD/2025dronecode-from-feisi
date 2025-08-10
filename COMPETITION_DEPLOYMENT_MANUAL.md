# 无人机竞赛部署完整手册
# Complete Drone Competition Deployment Manual

## 目录 Table of Contents

1. [系统概述](#1-系统概述)
2. [YOLO模型训练](#2-yolo模型训练)
3. [硬件配置与校准](#3-硬件配置与校准)
4. [系统集成与测试](#4-系统集成与测试)
5. [性能优化调试](#5-性能优化调试)
6. [竞赛日清单](#6-竞赛日清单)
7. [故障排除指南](#7-故障排除指南)
8. [维护与升级](#8-维护与升级)

---

## 1. 系统概述

### 1.1 当前实现状态
✅ **已完成的功能:**
- 完整的行为树框架和竞赛动作节点
- 5种障碍类型的自主穿越算法
- 配置文件和启动脚本
- 基础的目标检测框架
- 安全监控和边界检查
- 系统文档和验证脚本

### 1.2 需要完成的部署任务
🔄 **待完成任务:**
- YOLO模型训练和优化
- 硬件集成和校准
- 现场环境适配
- 系统性能调优
- 鲁棒性测试
- 备份方案准备

---

## 2. YOLO模型训练

### 2.1 数据收集要求

#### 2.1.1 障碍物数据收集
需要收集以下类型的标注数据:

**使用提供的数据收集工具:**
```bash
# 启动数据收集工具
python3 scripts/collect_data.py --output-dir training_data --camera-id 0

# 操作说明：
# - 数字键 0-4: 选择类别
# - 鼠标拖拽: 绘制边界框  
# - 空格键: 保存当前标注
# - ESC键: 清除当前标注框
# - Q键: 退出程序
```

**obs1 & obs2 - 框架检测 (Frame Detection)**
```yaml
类别: 
  - square_frame: 方形框架
  - circle_frame: 圆形框架
  
数据要求:
  - 总数量: 每类至少2000张图片
  - 角度覆盖: 0°-360° 全方位
  - 距离范围: 1m-10m
  - 光照条件: 室内/室外/阴天/晴天
  - 背景环境: 比赛场地类似背景
```

**obs5 - 刀旗检测 (Flag Detection)**
```yaml
类别:
  - red_flag: 红色刀旗
  - blue_flag: 蓝色刀旗  
  - green_flag: 绿色刀旗
  
数据要求:
  - 总数量: 每类至少1500张图片
  - 距离范围: 2m-15m
  - 角度变化: 考虑旗帜飘动
  - 遮挡情况: 部分遮挡样本占20%
```

#### 2.1.2 数据标注标准

**标注工具推荐:** LabelImg, CVAT, 或 Roboflow

**标注规范:**
```python
# 框架标注规范
class_mapping = {
    0: 'square_frame',    # 方形框架 - 标注整个可穿越区域
    1: 'circle_frame',    # 圆形框架 - 标注圆形边界
    2: 'red_flag',        # 红色刀旗 - 标注旗面区域
    3: 'blue_flag',       # 蓝色刀旗
    4: 'green_flag'       # 绿色刀旗
}

# 标注质量要求
annotation_requirements = {
    'bbox_accuracy': 0.95,      # 边界框准确度
    'consistency': 0.98,        # 标注一致性
    'completeness': 1.0         # 完整性(无漏标)
}
```

### 2.2 模型训练流程

#### 2.2.1 环境准备
```bash
# 创建训练环境
conda create -n yolo_training python=3.8
conda activate yolo_training

# 安装依赖
pip install ultralytics torch torchvision opencv-python
pip install roboflow supervision

# 克隆YOLOv8训练代码
git clone https://github.com/ultralytics/ultralytics.git
cd ultralytics
```

#### 2.2.2 数据集准备
```bash
# 创建数据集目录结构
mkdir -p competition_dataset/{train,val,test}/{images,labels}

# 数据集配置文件
# 创建 competition_dataset/dataset.yaml
```

**dataset.yaml配置:**
```yaml
# 竞赛数据集配置
path: /path/to/competition_dataset
train: train/images  
val: val/images
test: test/images

# 类别定义
nc: 5  # number of classes
names:
  0: square_frame
  1: circle_frame  
  2: red_flag
  3: blue_flag
  4: green_flag

# 数据增强设置
augment:
  hsv_h: 0.015      # 色调
  hsv_s: 0.7        # 饱和度
  hsv_v: 0.4        # 亮度
  degrees: 45.0     # 旋转角度
  translate: 0.2    # 平移
  scale: 0.5        # 缩放
  mosaic: 1.0       # 马赛克增强
  mixup: 0.1        # mixup增强
```

#### 2.2.3 模型训练命令
```bash
# 使用提供的训练脚本
python3 scripts/train_yolo.py \
    --config scripts/yolo_training_config.yaml \
    --model yolov8s \
    --export \
    --benchmark

# 训练配置文件说明:
# scripts/yolo_training_config.yaml - 完整的训练参数配置
# 包含数据增强、学习率调度、验证要求等所有设置
```

#### 2.2.4 模型验证和选择
```bash
# 模型性能评估
python val.py \
    --weights runs/train/competition_v1/weights/best.pt \
    --data competition_dataset/dataset.yaml \
    --imgsz 640

# 推理速度测试
python benchmark.py \
    --weights runs/train/competition_v1/weights/best.pt \
    --imgsz 640 \
    --device 0
```

**模型选择标准:**
```python
model_requirements = {
    'mAP50': 0.85,           # 精度要求
    'inference_time': 50,     # 推理时间 <50ms
    'model_size': 20,         # 模型大小 <20MB
    'confidence_stability': 0.9  # 置信度稳定性
}
```

### 2.3 模型部署优化

#### 2.3.1 模型转换
```bash
# 转换为ONNX格式 (推荐)
python export.py \
    --weights runs/train/competition_v2/weights/best.pt \
    --include onnx \
    --imgsz 640 \
    --device cpu

# 转换为TensorRT (NVIDIA GPU)
python export.py \
    --weights runs/train/competition_v2/weights/best.pt \
    --include engine \
    --imgsz 640 \
    --device 0
```

#### 2.3.2 集成到ROS系统
```python
# 更新 src/object_det/scripts/det.py
class CompetitionYOLO:
    def __init__(self, model_path, device='cpu'):
        self.model = YOLO(model_path)
        self.device = device
        self.class_names = {
            0: 'square_frame',
            1: 'circle_frame', 
            2: 'red_flag',
            3: 'blue_flag',
            4: 'green_flag'
        }
    
    def detect_obstacles(self, image):
        results = self.model(image, device=self.device)
        detections = []
        
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    cls_id = int(box.cls)
                    conf = float(box.conf)
                    if conf > 0.7:  # 置信度阈值
                        detection = {
                            'class': self.class_names[cls_id],
                            'confidence': conf,
                            'bbox': box.xyxy[0].cpu().numpy(),
                            'center': self.get_bbox_center(box.xyxy[0])
                        }
                        detections.append(detection)
        return detections
```

---

## 3. 硬件配置与校准

### 3.1 硬件需求清单

#### 3.1.1 核心硬件配置
```yaml
飞行平台:
  机型: DJI M300 RTK / Autel EVO II 或同等性能
  飞行时间: ≥25分钟
  负载能力: ≥1kg
  抗风能力: ≥8m/s

计算平台:
  主控: NVIDIA Jetson Xavier NX / AGX Xavier
  内存: ≥8GB RAM
  存储: ≥128GB NVMe SSD
  计算能力: ≥21 TOPS (AI)

传感器系统:
  相机: 
    - 主相机: 1080p 60fps, FOV 90°
    - 备用相机: 720p 30fps (可选)
  IMU: 高精度惯性测量单元
  激光雷达: Livox MID-360 (可选)
  GPS: RTK双频GPS模块

通信系统:
  数传: 2.4GHz/5.8GHz双频
  遥控: 支持手动接管
  地面站: 实时监控和调试
```

#### 3.1.2 软件环境要求
```bash
# 操作系统
Ubuntu 20.04 LTS (推荐)
ROS Noetic

# 关键软件包
- MAVROS (无人机通信)
- OpenCV 4.5+ (图像处理)
- CUDA 11.4+ (GPU加速)
- TensorRT 8+ (推理优化)

# Python环境
Python 3.8+
ultralytics==8.0.0
torch>=1.12.0
torchvision>=0.13.0
```

### 3.2 硬件校准流程

#### 3.2.1 相机标定
```bash
# 棋盘格标定
rosrun camera_calibration cameracalibrator.py \
    --size 8x6 \
    --square 0.025 \
    image:=/camera/image_raw

# 保存标定参数到
# src/sensor_pkg/config/camera_calibration.yaml
```

**相机参数文件示例:**
```yaml
camera_matrix:
  data: [615.123, 0.0, 320.0, 
         0.0, 615.123, 240.0,
         0.0, 0.0, 1.0]
distortion_coefficients:
  data: [-0.12, 0.05, 0.0, 0.0, 0.0]
image_width: 640
image_height: 480
```

#### 3.2.2 IMU校准
```bash
# 启动IMU校准
rosrun mavros mavros_imu_calibration

# 按照提示进行6面校准
# 校准参数自动保存到飞控
```

#### 3.2.3 飞控参数配置
```bash
# 使用QGroundControl配置飞控参数
# 关键参数设置:

# 位置控制器参数
MPC_XY_P: 0.95        # 水平位置增益
MPC_Z_P: 1.0          # 垂直位置增益
MPC_XY_VEL_P: 0.09    # 水平速度增益
MPC_Z_VEL_P: 0.2      # 垂直速度增益

# 速度限制
MPC_XY_VEL_MAX: 3.0   # 最大水平速度
MPC_Z_VEL_MAX_UP: 3.0 # 最大上升速度
MPC_Z_VEL_MAX_DN: 1.5 # 最大下降速度

# 安全参数
RTL_RETURN_ALT: 30.0  # 返航高度
LNDMC_LOW_T_THR: 0.3  # 降落检测
```

### 3.3 传感器集成

#### 3.3.1 多传感器融合配置
```yaml
# src/sensor_pkg/config/sensor_fusion.yaml
sensor_fusion:
  primary_sensors:
    - camera: /camera/image_raw
    - imu: /mavros/imu/data
    - gps: /mavros/global_position/global
    
  secondary_sensors:
    - lidar: /livox/lidar_3d (可选)
    - ultrasonic: /mavros/distance_sensor/hrlv20_pub
    
  fusion_parameters:
    position_trust_camera: 0.7
    position_trust_gps: 0.3
    orientation_trust_imu: 0.9
    orientation_trust_visual: 0.1
```

#### 3.3.2 传感器数据验证
```bash
# 验证传感器数据流
rostopic hz /camera/image_raw        # 应该 ≥20Hz
rostopic hz /mavros/imu/data         # 应该 ≥50Hz
rostopic hz /mavros/local_position/pose  # 应该 ≥20Hz

# 检查数据质量
rqt_plot /mavros/imu/data/angular_velocity/x:y:z
rqt_plot /mavros/local_position/pose/position/x:y:z
```

---

## 4. 系统集成与测试

### 4.1 软件集成流程

#### 4.1.1 代码编译与部署
```bash
# 使用自动部署脚本
./scripts/deploy_competition.sh

# 脚本功能:
# 1. 检查系统依赖
# 2. 编译ROS工作空间
# 3. 配置硬件参数
# 4. 训练YOLO模型 (可选)
# 5. 运行系统测试
# 6. 创建部署包
# 7. 生成检查清单

# 手动安装依赖
pip install -r requirements.txt
rosdep install --from-paths src --ignore-src -r -y
```

#### 4.1.2 配置文件调整
```bash
# 根据实际硬件调整配置
# 编辑 src/mission_pkg/config/competition_params.yaml

# 相机话题
vision:
  camera_topic: "/camera/image_raw"           # 根据实际相机话题
  
# 检测阈值
detection:
  confidence_threshold: 0.75                  # 根据模型性能调整
  
# 控制参数
control:
  position_tolerance: 0.2                     # 根据飞行器精度
  velocity_max: 2.5                          # 根据飞行器性能
  
# 安全参数  
safety:
  field_boundary: [10.0, 10.0, 4.0]         # 根据比赛场地
```

### 4.2 分模块测试

#### 4.2.1 视觉检测模块测试
```bash
# 1. 单独测试目标检测
rosrun object_det det.py

# 2. 在另一个终端发布测试图像
rostopic pub /camera/image_raw sensor_msgs/Image [图像数据]

# 3. 检查检测结果
rostopic echo /object_detection/results

# 4. 使用rqt查看结果
rqt_image_view /object_detection/visualization
```

**检测性能验证标准:**
```python
detection_performance_requirements = {
    'frame_detection_accuracy': 0.85,    # 框架检测准确率
    'flag_detection_accuracy': 0.80,     # 旗帜检测准确率
    'detection_latency': 0.05,           # 检测延迟 <50ms
    'false_positive_rate': 0.1,          # 误检率 <10%
    'detection_range': {
        'min_distance': 1.0,              # 最小检测距离
        'max_distance': 10.0              # 最大检测距离
    }
}
```

#### 4.2.2 行为树测试
```bash
# 1. 测试基础行为树功能
roslaunch mission_pkg test.launch

# 2. 测试单个动作节点
roslaunch mission_pkg single_cross_frame.launch

# 3. 测试完整竞赛流程
roslaunch mission_pkg competition.launch use_sim:=true debug_mode:=true
```

#### 4.2.3 飞控通信测试
```bash
# 1. 检查MAVROS连接
rosrun mavros mavcmd long 400 1 5 0 0 0 0 0

# 2. 测试解锁和锁定
rosservice call /mavros/cmd/arming "value: true"
rosservice call /mavros/cmd/arming "value: false"

# 3. 测试模式切换
rosservice call /mavros/set_mode "custom_mode: 'OFFBOARD'"
```

### 4.3 集成测试

#### 4.3.1 室内测试环境搭建
```yaml
测试环境要求:
  空间大小: 至少 6m × 6m × 3m
  障碍物设置:
    - 方形框架: 1.5m × 1.5m, 高度2m
    - 圆形框架: 直径1.6m, 高度2m  
    - 刀旗: 高度1.5m, 不同颜色
  安全防护:
    - 防护网设置
    - 紧急停止开关
    - 人员安全区域
```

#### 4.3.2 测试流程
```bash
# 阶段1: 静态测试 (无飞行)
# 1. 启动所有节点
roslaunch mission_pkg competition.launch use_sim:=false debug_mode:=true

# 2. 检查节点状态
rostopic list | grep competition
rosnode list | grep mission

# 3. 验证话题通信
rostopic echo /competition/state
rostopic echo /mavros/state

# 阶段2: 悬停测试
# 1. 手动起飞到安全高度
# 2. 切换到OFFBOARD模式
# 3. 测试位置保持能力

# 阶段3: 简单动作测试
# 测试单个障碍动作
roslaunch mission_pkg single_cross_frame.launch

# 阶段4: 完整流程测试
# 运行完整竞赛序列
```

#### 4.3.3 性能基准测试
```python
# 测试脚本: test_performance.py
import rospy
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class PerformanceTest:
    def __init__(self):
        self.start_time = None
        self.waypoint_times = []
        self.detection_times = []
        
    def run_benchmark(self):
        """运行性能基准测试"""
        tests = [
            self.test_takeoff_time,
            self.test_obstacle_detection_time,
            self.test_trajectory_following_accuracy,
            self.test_landing_precision
        ]
        
        results = {}
        for test in tests:
            result = test()
            results[test.__name__] = result
            
        return results
    
    def test_takeoff_time(self):
        """测试起飞时间"""
        # 从地面到2米高度的时间
        target_time = 8.0  # 应小于8秒
        return self.measure_action_time('takeoff', target_time)
```

---

## 5. 性能优化调试

### 5.1 系统性能监控

#### 5.1.1 实时监控工具
```bash
# 启动竞赛系统监控
python3 scripts/competition_monitor.py

# 监控功能:
# - 实时系统资源监控 (CPU, 内存, GPU)
# - 电池状态监控
# - 飞行轨迹记录
# - 目标检测性能统计
# - 异常情况自动告警

# 交互命令:
# Enter: 显示当前状态
# 'r': 生成监控报告
# 'q': 退出监控
```

#### 5.1.2 性能分析工具
```python
# 性能分析脚本
import psutil
import time
import rospy

class PerformanceMonitor:
    def __init__(self):
        self.start_time = time.time()
        
    def monitor_system(self):
        while not rospy.is_shutdown():
            # CPU使用率
            cpu_percent = psutil.cpu_percent(interval=1)
            
            # 内存使用
            memory = psutil.virtual_memory()
            
            # GPU使用 (如果有NVIDIA GPU)
            try:
                import pynvml
                pynvml.nvmlInit()
                handle = pynvml.nvmlDeviceGetHandleByIndex(0)
                gpu_util = pynvml.nvmlDeviceGetUtilizationRates(handle)
                gpu_memory = pynvml.nvmlDeviceGetMemoryInfo(handle)
            except:
                gpu_util = None
                gpu_memory = None
            
            # 记录性能数据
            self.log_performance(cpu_percent, memory, gpu_util, gpu_memory)
            time.sleep(1)
```

### 5.2 检测算法优化

#### 5.2.1 检测速度优化
```python
# 优化检测参数
detection_optimization = {
    # 输入图像尺寸优化
    'input_size': 416,              # 从640降到416提高速度
    
    # 推理优化
    'half_precision': True,         # 使用FP16半精度
    'batch_size': 1,               # 单张图像推理
    'conf_threshold': 0.6,         # 适当降低置信度阈值
    'nms_threshold': 0.45,         # NMS阈值优化
    
    # 多线程优化
    'num_threads': 4,              # CPU线程数
    'pin_memory': True,            # 固定内存
}

# 实现异步检测
import threading
import queue

class AsyncDetector:
    def __init__(self, model_path):
        self.model = YOLO(model_path)
        self.image_queue = queue.Queue(maxsize=2)
        self.result_queue = queue.Queue(maxsize=2)
        self.detection_thread = threading.Thread(target=self._detection_worker)
        self.detection_thread.start()
    
    def _detection_worker(self):
        while True:
            if not self.image_queue.empty():
                image = self.image_queue.get()
                result = self.model(image, imgsz=416, half=True)
                self.result_queue.put(result)
```

#### 5.2.2 检测精度优化
```python
# 多尺度检测
def multi_scale_detection(image, model, scales=[416, 512, 640]):
    """多尺度检测提高精度"""
    all_detections = []
    
    for scale in scales:
        detections = model(image, imgsz=scale)
        all_detections.extend(detections)
    
    # 合并和过滤重复检测
    final_detections = non_max_suppression_multi_scale(all_detections)
    return final_detections

# 置信度动态调整
class AdaptiveConfidence:
    def __init__(self):
        self.base_confidence = 0.7
        self.detection_history = []
        
    def get_dynamic_threshold(self, detection_context):
        """根据检测上下文动态调整置信度"""
        if detection_context['distance'] > 8.0:
            return self.base_confidence - 0.1  # 远距离降低阈值
        elif detection_context['lighting'] == 'poor':
            return self.base_confidence - 0.05  # 光照不好降低阈值
        else:
            return self.base_confidence
```

### 5.3 控制系统调优

#### 5.3.1 PID参数优化
```python
# PID调参工具
class PIDTuner:
    def __init__(self):
        self.position_pid = {'P': 0.8, 'I': 0.1, 'D': 0.2}
        self.velocity_pid = {'P': 1.2, 'I': 0.05, 'D': 0.1}
        
    def auto_tune_position_controller(self, flight_data):
        """基于飞行数据自动调参"""
        # 分析振荡情况
        oscillation = self.analyze_oscillation(flight_data['position'])
        
        if oscillation > 0.1:
            # 减小P增益
            self.position_pid['P'] *= 0.9
            self.position_pid['D'] *= 1.1
        elif oscillation < 0.05:
            # 增加P增益
            self.position_pid['P'] *= 1.05
            
        return self.position_pid
```

#### 5.3.2 轨迹优化
```python
# 轨迹平滑算法
import numpy as np
from scipy.interpolate import BSpline

class TrajectoryOptimizer:
    def __init__(self):
        self.smoothing_factor = 0.1
        
    def smooth_trajectory(self, waypoints, max_velocity=2.0, max_acceleration=1.0):
        """生成平滑轨迹"""
        # 使用B样条平滑
        t = np.linspace(0, 1, len(waypoints))
        
        # 分别处理x, y, z坐标
        x_coords = [wp[0] for wp in waypoints]
        y_coords = [wp[1] for wp in waypoints]
        z_coords = [wp[2] for wp in waypoints]
        
        # 创建B样条
        degree = min(3, len(waypoints) - 1)
        knots = np.linspace(0, 1, len(waypoints) + degree + 1)
        
        spline_x = BSpline(knots, x_coords, degree)
        spline_y = BSpline(knots, y_coords, degree)
        spline_z = BSpline(knots, z_coords, degree)
        
        # 生成平滑轨迹点
        num_points = len(waypoints) * 10
        t_smooth = np.linspace(0, 1, num_points)
        
        smooth_trajectory = []
        for t_val in t_smooth:
            point = [spline_x(t_val), spline_y(t_val), spline_z(t_val)]
            smooth_trajectory.append(point)
            
        return self.velocity_constrained_trajectory(smooth_trajectory, max_velocity)
```

---

## 6. 竞赛日清单

### 6.1 设备准备清单

#### 6.1.1 主要设备
```yaml
飞行器系统:
  - [ ] 无人机主体 (已校准)
  - [ ] 备用螺旋桨 (至少2套)
  - [ ] 电池 (主电池2块 + 备用2块)
  - [ ] 充电器和电源
  - [ ] 遥控器 (已对频)

计算设备:
  - [ ] 机载计算机 (Jetson等)
  - [ ] 地面站笔记本
  - [ ] 网络设备 (路由器/数传)
  - [ ] 备用SD卡/存储设备

传感器设备:
  - [ ] 相机模块 (已标定)
  - [ ] 激光雷达 (如果使用)
  - [ ] GPS模块
  - [ ] 备用传感器

工具设备:
  - [ ] 螺丝刀工具包
  - [ ] 万用表
  - [ ] 网线/USB线
  - [ ] 胶带和绑带
```

#### 6.1.2 软件准备
```bash
# 软件检查清单
- [ ] 系统镜像备份
- [ ] 最新代码版本
- [ ] 训练好的YOLO模型
- [ ] 配置文件 (针对比赛场地)
- [ ] 测试脚本和日志

# 软件版本确认
git log --oneline -5          # 确认代码版本
rostopic list                 # 确认ROS话题
rosnode list                  # 确认ROS节点
```

### 6.2 现场测试流程

#### 6.2.1 到场检查 (T-60分钟)
```bash
# 1. 硬件检查
./scripts/hardware_check.sh

# 2. 软件系统检查  
roslaunch mission_pkg system_check.launch

# 3. 通信检查
rostopic hz /mavros/state     # 检查飞控通信
rostopic hz /camera/image_raw # 检查相机数据

# 4. 环境适配
python scripts/environment_calibration.py
```

#### 6.2.2 场地标定 (T-45分钟)
```python
# 场地测量脚本
import numpy as np

class FieldCalibration:
    def __init__(self):
        self.obstacle_positions = {}
        
    def measure_field(self):
        """测量实际场地尺寸和障碍位置"""
        print("开始场地标定...")
        print("请使用激光测距仪测量以下位置:")
        
        obstacles = ['obs1_square', 'obs2_circle', 'obs3_double', 'obs4_somersault', 'obs5_flag']
        
        for obs in obstacles:
            x = float(input(f"输入 {obs} X坐标 (m): "))
            y = float(input(f"输入 {obs} Y坐标 (m): "))
            z = float(input(f"输入 {obs} Z坐标 (m): "))
            self.obstacle_positions[obs] = [x, y, z]
            
        self.update_config_file()
        
    def update_config_file(self):
        """更新配置文件"""
        config_file = "src/mission_pkg/config/competition_params.yaml"
        # 自动更新配置文件
        print(f"配置文件已更新: {config_file}")
```

#### 6.2.3 系统预飞测试 (T-30分钟)
```bash
# 测试流程
echo "开始系统预飞测试..."

# 1. 手动起飞测试
echo "1. 手动起飞到2米高度"
rosservice call /mavros/cmd/takeoff "altitude: 2.0"

# 2. 悬停稳定性测试
echo "2. 测试悬停稳定性 (30秒)"
sleep 30

# 3. 简单机动测试
echo "3. 测试基本机动"
roslaunch mission_pkg test_basic_maneuver.launch

# 4. 视觉检测测试
echo "4. 测试目标检测"
rostopic echo /object_detection/results --bags=1

# 5. 安全降落测试
echo "5. 测试安全降落"
rosservice call /mavros/cmd/land
```

### 6.3 比赛执行流程

#### 6.3.1 比赛准备 (T-10分钟)
```bash
# 最终系统检查
echo "=== 最终系统检查 ==="

# 1. 电池电量检查
echo "电池电量检查..."
rostopic echo /mavros/battery --bags=1

# 2. GPS信号检查
echo "GPS信号检查..."
rostopic echo /mavros/global_position/global --bags=1

# 3. 相机图像检查
echo "相机图像检查..."
rqt_image_view /camera/image_raw &

# 4. 系统状态最终确认
echo "系统状态最终确认..."
rosrun mission_pkg system_status_check.py
```

#### 6.3.2 比赛执行
```bash
# 启动竞赛任务
echo "启动竞赛任务..."
roslaunch mission_pkg competition.launch use_sim:=false debug_mode:=false

# 监控脚本
python scripts/competition_monitor.py &

# 日志记录
rosbag record -O competition_$(date +%Y%m%d_%H%M%S).bag \
    /mavros/local_position/pose \
    /mavros/state \
    /object_detection/results \
    /competition/state \
    /competition/status
```

#### 6.3.3 应急预案
```python
# 应急处理脚本
class EmergencyHandler:
    def __init__(self):
        self.emergency_actions = {
            'communication_lost': self.handle_comm_lost,
            'detection_failed': self.handle_detection_failed, 
            'control_unstable': self.handle_control_unstable,
            'battery_low': self.handle_battery_low
        }
    
    def handle_comm_lost(self):
        """通信丢失处理"""
        print("通信丢失 - 激活RTL模式")
        os.system("rosservice call /mavros/set_mode 'custom_mode: RTL'")
        
    def handle_detection_failed(self):
        """检测失败处理"""
        print("检测失败 - 切换手动模式")
        os.system("rosservice call /mavros/set_mode 'custom_mode: MANUAL'")
        
    def handle_control_unstable(self):
        """控制不稳定处理"""
        print("控制不稳定 - 紧急悬停")
        os.system("rostopic pub /mavros/setpoint_raw/local mavros_msgs/PositionTarget ...")
        
    def handle_battery_low(self):
        """电量不足处理"""
        print("电量不足 - 立即降落")
        os.system("rosservice call /mavros/cmd/land")
```

---

## 7. 故障排除指南

### 7.1 常见问题诊断

#### 7.1.1 检测系统问题
```python
# 检测问题诊断脚本
class DetectionDiagnostics:
    def __init__(self):
        self.diagnostic_tests = [
            self.test_camera_connection,
            self.test_model_loading,
            self.test_detection_accuracy,
            self.test_detection_latency
        ]
    
    def run_full_diagnosis(self):
        """运行完整诊断"""
        results = {}
        for test in self.diagnostic_tests:
            try:
                result = test()
                results[test.__name__] = {'status': 'PASS', 'details': result}
            except Exception as e:
                results[test.__name__] = {'status': 'FAIL', 'error': str(e)}
        
        return results
    
    def test_camera_connection(self):
        """测试相机连接"""
        import cv2
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            raise Exception("无法打开相机")
        
        ret, frame = cap.read()
        if not ret:
            raise Exception("无法读取相机图像")
            
        cap.release()
        return f"相机分辨率: {frame.shape}"
    
    def test_model_loading(self):
        """测试模型加载"""
        from ultralytics import YOLO
        model_path = "src/object_det/scripts/Model/competition.pt"
        
        if not os.path.exists(model_path):
            raise Exception(f"模型文件不存在: {model_path}")
            
        model = YOLO(model_path)
        return f"模型加载成功: {model_path}"
```

#### 7.1.2 飞控通信问题
```bash
# 飞控通信诊断
echo "=== 飞控通信诊断 ==="

# 1. 检查串口连接
echo "检查串口设备..."
ls -la /dev/ttyACM* /dev/ttyUSB*

# 2. 检查MAVROS连接
echo "检查MAVROS连接状态..."
rostopic echo /mavros/state --bags=1

# 3. 检查心跳包
echo "检查心跳包..."
rostopic hz /mavros/state

# 4. 测试基本指令
echo "测试基本指令..."
rosservice call /mavros/cmd/arming "value: false"
```

#### 7.1.3 行为树执行问题
```python
# 行为树诊断工具
class BehaviorTreeDiagnostics:
    def __init__(self):
        self.bt_subscriber = rospy.Subscriber('/bt_status', String, self.bt_status_callback)
        self.current_node = None
        self.node_history = []
        
    def bt_status_callback(self, msg):
        """行为树状态回调"""
        self.current_node = msg.data
        self.node_history.append((time.time(), msg.data))
        
    def diagnose_stuck_node(self):
        """诊断卡住的节点"""
        if len(self.node_history) < 2:
            return "历史数据不足"
            
        last_node = self.node_history[-1]
        time_in_current = time.time() - last_node[0]
        
        if time_in_current > 30:  # 超过30秒在同一节点
            return f"节点可能卡住: {last_node[1]}, 时间: {time_in_current:.1f}s"
        
        return "节点执行正常"
```

### 7.2 性能问题解决

#### 7.2.1 延迟问题优化
```python
# 延迟分析工具
class LatencyAnalyzer:
    def __init__(self):
        self.timestamps = {}
        
    def measure_pipeline_latency(self):
        """测量处理管道延迟"""
        # 图像采集到检测结果的延迟
        image_timestamp = rospy.Time.now()
        
        # 模拟检测过程
        detection_result = self.run_detection()
        detection_timestamp = rospy.Time.now()
        
        # 计算延迟
        latency = (detection_timestamp - image_timestamp).to_sec()
        
        if latency > 0.1:  # 超过100ms
            print(f"警告: 检测延迟过高 {latency*1000:.1f}ms")
            return self.optimize_detection_pipeline()
        
        return latency
    
    def optimize_detection_pipeline(self):
        """优化检测管道"""
        optimizations = [
            "降低输入图像分辨率",
            "使用TensorRT加速", 
            "启用GPU推理",
            "减少检测频率",
            "使用异步处理"
        ]
        
        print("建议的优化措施:")
        for i, opt in enumerate(optimizations, 1):
            print(f"{i}. {opt}")
```

#### 7.2.2 内存问题处理
```python
# 内存监控和优化
import psutil
import gc

class MemoryOptimizer:
    def __init__(self):
        self.memory_threshold = 80  # 内存使用率阈值(%)
        
    def monitor_memory(self):
        """监控内存使用"""
        memory = psutil.virtual_memory()
        if memory.percent > self.memory_threshold:
            print(f"内存使用率过高: {memory.percent:.1f}%")
            self.optimize_memory()
            
    def optimize_memory(self):
        """内存优化"""
        # 强制垃圾回收
        gc.collect()
        
        # 清理图像缓存
        if hasattr(self, 'image_cache'):
            self.image_cache.clear()
            
        # 限制检测历史记录
        if hasattr(self, 'detection_history'):
            self.detection_history = self.detection_history[-100:]  # 只保留最近100条
            
        print("内存优化完成")
```

### 7.3 紧急恢复程序

#### 7.3.1 系统重启程序
```bash
#!/bin/bash
# emergency_restart.sh - 紧急重启脚本

echo "=== 紧急系统重启程序 ==="

# 1. 安全降落无人机
echo "1. 安全降落无人机..."
rosservice call /mavros/cmd/land
sleep 10

# 2. 停止所有ROS节点
echo "2. 停止ROS节点..."
rosnode kill -a
sleep 3

# 3. 重启核心服务
echo "3. 重启核心服务..."
sudo systemctl restart ros
sleep 5

# 4. 重新启动系统
echo "4. 重新启动竞赛系统..."
source ~/catkin_ws/devel/setup.bash
roslaunch mission_pkg competition.launch use_sim:=false debug_mode:=true

echo "系统重启完成"
```

#### 7.3.2 数据恢复程序
```python
# 数据恢复脚本
import os
import shutil
from datetime import datetime

class DataRecovery:
    def __init__(self):
        self.backup_dir = "/backup/competition_data"
        self.recovery_dir = "/tmp/recovery"
        
    def create_emergency_backup(self):
        """创建紧急备份"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_path = os.path.join(self.backup_dir, f"emergency_{timestamp}")
        
        # 备份关键文件
        critical_files = [
            "src/mission_pkg/config/",
            "src/object_det/scripts/Model/",
            "logs/",
            "rosbag/"
        ]
        
        os.makedirs(backup_path, exist_ok=True)
        
        for file_path in critical_files:
            if os.path.exists(file_path):
                dest_path = os.path.join(backup_path, os.path.basename(file_path))
                if os.path.isdir(file_path):
                    shutil.copytree(file_path, dest_path)
                else:
                    shutil.copy2(file_path, dest_path)
                    
        print(f"紧急备份创建完成: {backup_path}")
        
    def restore_from_backup(self, backup_name):
        """从备份恢复"""
        backup_path = os.path.join(self.backup_dir, backup_name)
        
        if not os.path.exists(backup_path):
            print(f"备份不存在: {backup_path}")
            return False
            
        # 恢复配置文件
        config_backup = os.path.join(backup_path, "config")
        if os.path.exists(config_backup):
            shutil.copytree(config_backup, "src/mission_pkg/config/", dirs_exist_ok=True)
            
        print(f"从备份恢复完成: {backup_name}")
        return True
```

---

## 8. 维护与升级

### 8.1 日常维护清单

#### 8.1.1 硬件维护
```yaml
每日维护:
  - [ ] 检查螺旋桨磨损情况
  - [ ] 清洁相机镜头
  - [ ] 检查电池电压和充电次数
  - [ ] 检查各连接线缆

每周维护:
  - [ ] 校准IMU和指南针
  - [ ] 检查GPS定位精度
  - [ ] 更新飞控固件
  - [ ] 备份系统配置

每月维护:
  - [ ] 深度清洁设备
  - [ ] 重新标定相机
  - [ ] 检查机械磨损
  - [ ] 更新系统软件
```

#### 8.1.2 软件维护
```bash
# 定期软件维护脚本
#!/bin/bash
# maintenance.sh

echo "=== 系统维护脚本 ==="

# 1. 更新系统包
echo "1. 更新系统包..."
sudo apt update && sudo apt upgrade -y

# 2. 清理日志文件
echo "2. 清理系统日志..."
sudo journalctl --vacuum-time=7d
find ~/.ros/log -name "*.log" -mtime +7 -delete

# 3. 备份配置文件
echo "3. 备份配置文件..."
DATE=$(date +%Y%m%d)
tar -czf backup_config_$DATE.tar.gz src/mission_pkg/config/

# 4. 检查磁盘空间
echo "4. 检查磁盘空间..."
df -h | grep -E "(/$|/home)"

# 5. 测试关键功能
echo "5. 测试系统功能..."
roslaunch mission_pkg system_test.launch
```

### 8.2 性能监控与分析

#### 8.2.1 长期性能监控
```python
# 性能监控和分析系统
import sqlite3
import matplotlib.pyplot as plt
from datetime import datetime, timedelta

class PerformanceAnalyzer:
    def __init__(self):
        self.db_path = "performance_data.db"
        self.init_database()
        
    def init_database(self):
        """初始化数据库"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS flight_performance (
                id INTEGER PRIMARY KEY,
                timestamp DATETIME,
                mission_type TEXT,
                completion_time REAL,
                detection_accuracy REAL,
                trajectory_error REAL,
                battery_consumption REAL,
                notes TEXT
            )
        ''')
        
        conn.commit()
        conn.close()
        
    def log_flight_performance(self, mission_data):
        """记录飞行性能数据"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            INSERT INTO flight_performance 
            (timestamp, mission_type, completion_time, detection_accuracy, 
             trajectory_error, battery_consumption, notes)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        ''', (
            datetime.now(),
            mission_data['mission_type'],
            mission_data['completion_time'],
            mission_data['detection_accuracy'],
            mission_data['trajectory_error'],
            mission_data['battery_consumption'],
            mission_data.get('notes', '')
        ))
        
        conn.commit()
        conn.close()
        
    def generate_performance_report(self, days=30):
        """生成性能报告"""
        conn = sqlite3.connect(self.db_path)
        
        # 查询最近30天的数据
        start_date = datetime.now() - timedelta(days=days)
        
        df = pd.read_sql_query('''
            SELECT * FROM flight_performance 
            WHERE timestamp > ?
            ORDER BY timestamp
        ''', conn, params=[start_date])
        
        conn.close()
        
        # 生成图表
        self.plot_performance_trends(df)
        
        return df
```

#### 8.2.2 故障预测分析
```python
# 故障预测系统
from sklearn.ensemble import IsolationForest
import numpy as np

class FaultPredictor:
    def __init__(self):
        self.anomaly_detector = IsolationForest(contamination=0.1)
        self.feature_history = []
        
    def extract_features(self, flight_data):
        """提取故障预测特征"""
        features = {
            'avg_battery_voltage': np.mean(flight_data['battery_voltage']),
            'max_vibration': np.max(flight_data['vibration']),
            'gps_accuracy_variance': np.var(flight_data['gps_accuracy']),
            'detection_success_rate': flight_data['detection_success_count'] / flight_data['total_detections'],
            'control_response_time': np.mean(flight_data['control_response_times']),
            'temperature_max': np.max(flight_data['component_temperatures'])
        }
        
        return np.array(list(features.values()))
        
    def predict_potential_failures(self, current_flight_data):
        """预测潜在故障"""
        current_features = self.extract_features(current_flight_data)
        
        if len(self.feature_history) > 10:
            # 训练异常检测模型
            self.anomaly_detector.fit(np.array(self.feature_history))
            
            # 检测当前数据是否异常
            anomaly_score = self.anomaly_detector.decision_function([current_features])[0]
            is_anomaly = self.anomaly_detector.predict([current_features])[0] == -1
            
            if is_anomaly:
                return {
                    'risk_level': 'HIGH' if anomaly_score < -0.5 else 'MEDIUM',
                    'anomaly_score': anomaly_score,
                    'recommended_actions': self.get_maintenance_recommendations(current_features)
                }
        
        # 记录当前特征
        self.feature_history.append(current_features)
        if len(self.feature_history) > 100:
            self.feature_history.pop(0)  # 保持固定长度
            
        return {'risk_level': 'LOW', 'anomaly_score': 0.0}
```

### 8.3 系统升级指南

#### 8.3.1 软件升级流程
```bash
# 系统升级脚本
#!/bin/bash
# system_upgrade.sh

echo "=== 系统升级流程 ==="

# 1. 创建升级前备份
echo "1. 创建系统备份..."
timestamp=$(date +%Y%m%d_%H%M%S)
backup_dir="/backup/upgrade_$timestamp"
mkdir -p $backup_dir

# 备份关键组件
tar -czf $backup_dir/catkin_ws.tar.gz ~/catkin_ws/
cp -r ~/catkin_ws/src/mission_pkg/config $backup_dir/
cp ~/.bashrc $backup_dir/

echo "备份完成: $backup_dir"

# 2. 下载新版本
echo "2. 下载新版本代码..."
cd ~/catkin_ws/src/
git fetch origin
git checkout -b backup_current  # 创建当前分支备份
git checkout main
git pull origin main

# 3. 检查依赖变化
echo "3. 检查并安装新依赖..."
rosdep update
rosdep install --from-paths . --ignore-src -r -y

# 4. 编译新版本
echo "4. 编译新版本..."
cd ~/catkin_ws/
catkin_make clean
catkin_make -DCMAKE_BUILD_TYPE=Release

# 5. 运行升级后测试
echo "5. 运行升级后测试..."
source devel/setup.bash
roslaunch mission_pkg system_test.launch

echo "升级完成！"
```

#### 8.3.2 模型升级流程
```python
# YOLO模型升级管理
class ModelUpgradeManager:
    def __init__(self):
        self.model_dir = "src/object_det/scripts/Model"
        self.current_model = "competition_v1.pt"
        self.backup_dir = "model_backups"
        
    def upgrade_model(self, new_model_path, validation_dataset):
        """升级YOLO模型"""
        print("开始模型升级流程...")
        
        # 1. 备份当前模型
        backup_path = self.backup_current_model()
        print(f"当前模型已备份至: {backup_path}")
        
        # 2. 验证新模型性能
        performance = self.validate_new_model(new_model_path, validation_dataset)
        
        if performance['mAP'] < 0.8:  # 性能阈值
            print(f"新模型性能不达标: mAP={performance['mAP']:.3f}")
            return False
            
        # 3. 部署新模型
        self.deploy_new_model(new_model_path)
        
        # 4. 运行集成测试
        test_result = self.run_integration_test()
        
        if not test_result:
            print("集成测试失败，回滚到备份模型")
            self.rollback_model(backup_path)
            return False
            
        print("模型升级成功！")
        return True
        
    def validate_new_model(self, model_path, validation_dataset):
        """验证新模型性能"""
        from ultralytics import YOLO
        
        model = YOLO(model_path)
        results = model.val(data=validation_dataset)
        
        return {
            'mAP': results.box.map,
            'mAP50': results.box.map50,
            'precision': results.box.mp,
            'recall': results.box.mr
        }
```

---

## 总结

本手册涵盖了从当前代码状态到竞赛部署的完整流程：

### 关键部署步骤总结:

1. **YOLO模型训练** - 收集标注数据，训练适合竞赛环境的检测模型
2. **硬件集成校准** - 配置传感器，校准系统参数  
3. **系统测试验证** - 分模块测试，集成测试，性能验证
4. **现场适配调优** - 场地测量，参数调整，性能优化
5. **竞赛执行** - 按照标准流程执行，实时监控，应急处理

### 成功部署的关键要素:

- **充分的数据准备**: 高质量的标注数据是检测精度的基础
- **系统性能优化**: 保证实时性能满足竞赛要求
- **全面的测试验证**: 模拟各种场景，确保系统鲁棒性
- **完善的应急预案**: 准备多种备份方案和故障恢复程序
- **持续监控分析**: 建立性能监控和故障预测机制

按照本手册的步骤和要求，可以将现有的代码框架成功部署到实际竞赛中。建议分阶段执行，每个阶段都要充分测试验证后再进入下一阶段。