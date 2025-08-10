# 竞赛部署快速指南
# Quick Competition Deployment Guide

本文档提供竞赛系统的快速部署流程，详细信息请参考 [完整部署手册](COMPETITION_DEPLOYMENT_MANUAL.md)。

## 🚀 一键部署

```bash
# 运行自动部署脚本
chmod +x scripts/deploy_competition.sh
./scripts/deploy_competition.sh
```

该脚本将自动完成：
- ✅ 系统依赖检查
- ✅ 代码编译配置  
- ✅ 硬件参数配置
- ✅ YOLO模型训练 (可选)
- ✅ 系统功能测试
- ✅ 部署包创建
- ✅ 检查清单生成

## 📋 关键步骤详解

### 1. 数据收集 (如需训练新模型)
```bash
# 启动数据收集工具
python3 scripts/collect_data.py --output-dir training_data

# 操作：鼠标拖拽标注，空格保存，Q键退出
# 目标：每类收集1500+张标注图片
```

### 2. 模型训练
```bash  
# 训练YOLO检测模型
python3 scripts/train_yolo.py \
    --config scripts/yolo_training_config.yaml \
    --model yolov8s \
    --export
```

### 3. 系统监控
```bash
# 启动实时监控
python3 scripts/competition_monitor.py

# 功能：实时状态显示，性能报告生成
```

### 4. 竞赛启动
```bash
# 仿真测试
roslaunch mission_pkg competition.launch use_sim:=true debug_mode:=true

# 实际比赛
roslaunch mission_pkg competition.launch use_sim:=false
```

## 📁 重要文件说明

| 文件/目录 | 作用 | 说明 |
|----------|------|------|
| `COMPETITION_DEPLOYMENT_MANUAL.md` | 完整部署手册 | 详细的部署和使用指南 |
| `scripts/deploy_competition.sh` | 自动部署脚本 | 一键完成系统部署 |
| `scripts/train_yolo.py` | YOLO训练脚本 | 自动训练检测模型 |
| `scripts/collect_data.py` | 数据收集工具 | 交互式数据标注 |
| `scripts/competition_monitor.py` | 系统监控工具 | 实时性能监控 |
| `requirements.txt` | Python依赖 | 所需Python包列表 |
| `validate_system.sh` | 系统验证脚本 | 检查系统完整性 |

## 🎯 竞赛关键参数

### 障碍类型配置
```yaml
# 在 src/mission_pkg/config/competition_params.yaml 中配置
obstacles:
  obs1_square_frame:    # 方框直穿
    position: [2.0, 0.0, 2.0]
  obs2_circle_frame:    # 圆框直穿  
    position: [4.0, 0.0, 2.0]
  obs3_double_frame:    # 双框穿越
    frame1_position: [6.0, -1.0, 2.0]
    frame2_position: [6.0, 1.0, 2.0]
  obs4_somersault:      # 翻跟斗
    lower_frame_position: [8.0, 0.0, 1.5]
    upper_frame_position: [8.0, 0.0, 2.5]
  obs5_flag_circle:     # 环绕刀旗
    flag_color: "red"
    circle_radius: 2.0
```

### 安全参数设置
```yaml
safety:
  field_boundary: [10.0, 10.0, 4.0]  # 场地边界
  emergency_stop_height: 0.5         # 紧急停止高度
  timeout_general: 30.0              # 通用超时
```

## ⚠️ 竞赛日检查清单

### T-60分钟：硬件检查
- [ ] 无人机硬件完好
- [ ] 电池电量充足
- [ ] 相机图像清晰
- [ ] 通信连接正常

### T-30分钟：系统测试
- [ ] 运行系统验证: `./validate_system.sh`
- [ ] 测试手动起飞
- [ ] 验证目标检测
- [ ] 确认安全降落

### T-10分钟：竞赛准备
- [ ] 测量障碍物位置并更新配置
- [ ] 启动监控系统
- [ ] 确认应急预案
- [ ] 启动竞赛任务

## 🛠️ 故障排除

### 常见问题
1. **编译错误**: 运行 `rosdep install --from-paths src --ignore-src -r -y`
2. **检测不准**: 重新训练模型或调整置信度阈值
3. **飞控连接**: 检查串口连接和权限
4. **性能问题**: 使用监控工具分析瓶颈

### 应急操作
```bash
# 紧急停止
rosservice call /mavros/cmd/arming "value: false"

# 强制降落  
rosservice call /mavros/cmd/land

# 重启系统
./scripts/emergency_restart.sh
```

## 📞 技术支持

遇到问题时：
1. 查看 `COMPETITION_DEPLOYMENT_MANUAL.md` 详细说明
2. 检查系统日志和监控报告
3. 运行 `validate_system.sh` 诊断问题
4. 准备备用方案和手动控制

---

**祝比赛顺利！🏆**

*更多详细信息请参考完整部署手册和系统设计文档。*