# 无人机自主飞行竞赛系统设计文档

## 系统概述

本系统基于行为树(Behavior Tree)框架设计，用于完成无人机自主飞行竞赛任务。系统能够自主完成5种不同类型的障碍穿越和机动动作。

## 竞赛要求

### 场地规格
- **尺寸**: 10m × 10m
- **限高**: 4m  
- **障碍高度范围**: 1m-3m

### 障碍类型及要求
| 障碍标识 | 要求 | 实现方案 |
|---------|------|----------|
| obs1 | 方框直穿 | 使用现有CrossFrame节点 |
| obs2 | 圆框直穿 | 使用现有CrossFrame节点 |
| obs3 | 左进右出/右进左出 | 新增DoubleFrame节点 |
| obs4 | 上下翻跟斗 | 新增Somersault节点 |
| obs5 | 环绕刀旗一圈 | 新增CircleFlag节点 |

## 系统架构

### 核心模块

#### 1. 任务调度器 (CompetitionScheduler)
- **功能**: 高层任务状态管理和调度
- **特点**: 
  - 状态机设计，安全监控
  - 超时保护，紧急停止
  - 任务进度跟踪

#### 2. 特技控制模块

##### 2.1 翻滚控制 (Somersault)
- **适用障碍**: obs4 (上下翻跟斗)
- **控制策略**: 
  - 分阶段状态机控制
  - 垂直平面圆形轨迹生成
  - 速度和位置双闭环控制
- **安全特性**: 超时保护、位置检查

##### 2.2 环绕控制 (CircleFlag)  
- **适用障碍**: obs5 (环绕刀旗)
- **控制策略**:
  - 目标检测与定位
  - 水平面圆周轨迹控制
  - 自适应朝向控制
- **安全特性**: 检测超时、环绕限制

##### 2.3 双框穿越 (DoubleFrame)
- **适用障碍**: obs3 (双框穿越)
- **控制策略**:
  - 轨迹点计算和规划
  - 分段式导航控制
  - 方向自适应选择

#### 3. 配置管理
- **参数文件**: `competition_params.yaml`
- **行为树配置**: `competition.xml` / `competition_with_scheduler.xml`
- **启动配置**: `competition.launch`

## 文件结构

```
src/mission_pkg/
├── config/
│   ├── competition.xml                    # 基础竞赛行为树
│   ├── competition_with_scheduler.xml     # 带调度器的行为树
│   └── competition_params.yaml           # 竞赛参数配置
├── launch/
│   └── competition.launch                # 竞赛启动文件
├── include/plugins/action/
│   ├── somersault.h                      # 翻滚动作头文件
│   ├── circle_flag.h                     # 环绕刀旗头文件
│   ├── double_frame.h                    # 双框穿越头文件
│   ├── competition_scheduler.h           # 任务调度器头文件
│   └── publish_obstacle_complete.h       # 障碍完成发布头文件
└── src/plugins/action/
    ├── somersault.cpp                    # 翻滚动作实现
    ├── circle_flag.cpp                   # 环绕刀旗实现
    ├── double_frame.cpp                  # 双框穿越实现
    ├── competition_scheduler.cpp         # 任务调度器实现
    └── publish_obstacle_complete.cpp     # 障碍完成发布实现
```

## 接口说明

### 1. 行为树节点接口

#### Somersault (翻滚控制)
```xml
<Somersault entry_pos="8.0;0.0;1.5"     # 下框入口位置 (x;y;z)
           exit_pos="8.0;0.0;2.5"       # 上框出口位置 (x;y;z)  
           loop_radius="1.0"            # 翻滚半径 (m)
           loop_speed="2.0" />          # 翻滚速度 (m/s)
```

#### CircleFlag (环绕刀旗)
```xml
<CircleFlag flag_color="red"             # 目标刀旗颜色
           search_area="9.0;0.0;2.0"    # 搜索区域中心 (x;y;z)
           circle_radius="2.0"          # 环绕半径 (m)
           circle_speed="1.5"           # 环绕速度 (m/s)
           circle_height="2.0" />       # 环绕高度 (m)
```

#### DoubleFrame (双框穿越)
```xml
<DoubleFrame first_frame="6.0;-1.0;2.0"  # 第一框位置 (x;y;z)
            second_frame="6.0;1.0;2.0"  # 第二框位置 (x;y;z)
            direction="left_right"      # 穿越方向
            approach_speed="1.0" />     # 接近速度 (m/s)
```

### 2. ROS话题接口

#### 发布话题
- `/competition/state` (std_msgs/String): 当前任务状态
- `/competition/status` (std_msgs/String): 任务状态描述
- `/competition/obstacle_complete` (std_msgs/Bool): 障碍完成标志
- `/competition/current_obstacle` (std_msgs/Int32): 当前障碍编号

#### 订阅话题  
- `/mavros/local_position/local` (geometry_msgs/PoseStamped): 无人机位置
- `/mavros/state` (mavros_msgs/State): 无人机状态
- `/object_detection/flag_detection` (std_msgs/String): 刀旗检测结果
- `/object_detection/frame_detection` (std_msgs/String): 框架检测结果

## 使用方法

### 1. 编译系统
```bash
cd /path/to/workspace
catkin_make
source devel/setup.bash
```

### 2. 启动竞赛任务
```bash
# 仿真环境
roslaunch mission_pkg competition.launch use_sim:=true debug_mode:=true

# 真实环境  
roslaunch mission_pkg competition.launch use_sim:=false debug_mode:=false
```

### 3. 参数调整
编辑 `config/competition_params.yaml` 文件调整：
- 障碍位置和尺寸
- 飞行速度和容差
- 安全参数和超时设置

## 关键特性

### 1. 安全保障
- **边界检查**: 实时监控飞行区域边界
- **高度限制**: 严格控制最大飞行高度
- **超时保护**: 各阶段设置合理超时时间
- **紧急停止**: 支持手动和自动紧急停止

### 2. 容错机制
- **状态恢复**: 异常情况下的状态恢复
- **重试机制**: 关键动作的重试逻辑
- **降级策略**: 复杂动作的简化降级

### 3. 调试支持
- **状态可视化**: RViz中的状态和轨迹显示
- **日志记录**: 详细的执行日志和数据记录
- **参数调试**: 运行时参数调整和优化

## 扩展建议

### 1. 增强感知能力
- 集成更精确的目标检测算法
- 添加激光雷达障碍检测
- 实现实时SLAM定位

### 2. 优化控制算法  
- 使用MPC进行轨迹跟踪优化
- 添加自适应控制参数
- 实现风干扰补偿

### 3. 提升鲁棒性
- 添加多传感器融合
- 实现故障检测与隔离
- 增强异常恢复能力

## 注意事项

1. **安全第一**: 在真实飞行前充分仿真测试
2. **参数调优**: 根据实际飞机特性调整控制参数  
3. **环境适应**: 考虑光照、风速等环境因素影响
4. **备份方案**: 准备手动控制等备份操作方案

## 技术支持

如有问题或需要技术支持，请：
1. 查看系统日志确定问题原因
2. 检查参数配置是否正确
3. 验证硬件连接和状态
4. 联系开发团队获取帮助