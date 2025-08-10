# 2025 Drone Competition Code

## 项目概述
本项目为2025年无人机自主飞行竞赛开发的完整代码框架，基于ROS和行为树实现自主导航、障碍穿越和特技飞行功能。

## 🚁 竞赛能力
系统支持完成以下竞赛障碍：
- **obs1**: 方框直穿 - 无人机需从方框中穿过
- **obs2**: 圆框直穿 - 无人机需从圆框中穿过  
- **obs3**: 双框穿越 - 左进右出/右进左出复杂机动
- **obs4**: 翻滚特技 - 上下翻跟斗动作
- **obs5**: 环绕飞行 - 环绕指定颜色刀旗一圈

## 🔧 系统架构
- **感知模块**: 基于YOLO的目标检测，LiDAR-惯性融合定位
- **规划模块**: Ego-planner路径规划，行为树任务调度
- **控制模块**: MAVROS飞控接口，视觉伺服控制
- **安全模块**: 边界检查，超时保护，紧急停止

## 🚀 快速开始

### 1. 环境准备
```bash
# 确保ROS环境
source /opt/ros/noetic/setup.bash

# 编译工作空间  
cd /path/to/workspace
catkin_make
source devel/setup.bash
```

### 2. 运行竞赛任务
```bash
# 仿真测试
roslaunch mission_pkg competition.launch use_sim:=true debug_mode:=true

# 实际飞行
roslaunch mission_pkg competition.launch use_sim:=false
```

### 3. 系统验证
```bash
# 运行验证脚本
./validate_system.sh
```

## 📚 详细文档
- [系统设计文档](COMPETITION_SYSTEM_DESIGN.md) - 完整技术设计说明
- [快速开始指南](QUICK_START_GUIDE.md) - 使用教程和示例代码

## 🏗️ 项目结构
```
├── src/mission_pkg/           # 主要任务包
│   ├── config/               # 配置文件
│   │   ├── competition.xml   # 竞赛行为树配置
│   │   └── competition_params.yaml  # 参数配置
│   ├── launch/               # 启动文件
│   ├── include/plugins/action/  # 动作节点头文件
│   └── src/plugins/action/   # 动作节点实现
├── src/object_det/           # 目标检测模块
├── src/ego-planner/          # 路径规划模块  
├── src/faster-lio-main/      # 定位模块
└── validate_system.sh        # 系统验证脚本
```

## 🎯 竞赛特色功能

### 翻滚控制 (Somersault)
实现垂直平面内的翻滚机动，支持参数化半径和速度控制。

### 环绕控制 (CircleFlag)  
基于视觉检测的刀旗环绕飞行，支持多种颜色目标识别。

### 双框穿越 (DoubleFrame)
复杂的双框架穿越轨迹规划，支持左右进出方向配置。

### 高级调度器 (CompetitionScheduler)
状态机驱动的任务调度，提供安全监控和故障恢复。

## ⚠️ 安全注意事项
1. **仿真先行**: 实际飞行前必须完成仿真验证
2. **参数调优**: 根据实际无人机特性调整控制参数
3. **边界设置**: 确保飞行区域边界配置正确
4. **应急预案**: 准备手动控制等应急操作

## 🤝 贡献
欢迎提交问题报告和改进建议。请确保：
- 代码符合项目规范
- 添加必要的测试和文档
- 通过系统验证脚本检查

## 📧 技术支持
如有技术问题，请查看文档或提交Issue。