#!/bin/bash
# 竞赛系统部署脚本
# Competition System Deployment Script

set -e  # 遇到错误立即退出

echo "=== 竞赛系统自动部署脚本 ==="
echo "Competition System Auto Deployment Script"
echo "========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 全局变量
WORKSPACE_ROOT=$(pwd)
LOG_FILE="deployment_$(date +%Y%m%d_%H%M%S).log"
COMPETITION_DATE=""
TEAM_NAME=""
HARDWARE_CONFIG=""

# 日志函数
log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a $LOG_FILE
}

log_success() {
    echo -e "${GREEN}✅ $1${NC}" | tee -a $LOG_FILE
}

log_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}" | tee -a $LOG_FILE
}

log_error() {
    echo -e "${RED}❌ $1${NC}" | tee -a $LOG_FILE
}

log_info() {
    echo -e "${BLUE}ℹ️  $1${NC}" | tee -a $LOG_FILE
}

# 检查依赖
check_dependencies() {
    log_info "检查系统依赖..."
    
    # 检查ROS
    if ! command -v roscore &> /dev/null; then
        log_error "ROS未安装或未正确配置"
        exit 1
    fi
    log_success "ROS环境正常"
    
    # 检查Python依赖
    python3 -c "import cv2, numpy, yaml, torch" 2>/dev/null || {
        log_error "Python依赖缺失，请运行: pip install -r requirements.txt"
        exit 1
    }
    log_success "Python依赖正常"
    
    # 检查CUDA (可选)
    if command -v nvidia-smi &> /dev/null; then
        log_success "NVIDIA GPU可用"
    else
        log_warning "未检测到NVIDIA GPU，将使用CPU"
    fi
}

# 系统初始化
initialize_system() {
    log_info "初始化竞赛系统..."
    
    # 创建必要目录
    mkdir -p logs/
    mkdir -p backup/
    mkdir -p training_data/
    mkdir -p models/
    
    # 编译ROS包
    log_info "编译ROS工作空间..."
    catkin_make -DCMAKE_BUILD_TYPE=Release
    if [ $? -eq 0 ]; then
        log_success "ROS编译成功"
    else
        log_error "ROS编译失败"
        exit 1
    fi
    
    # 设置环境变量
    source devel/setup.bash
    log_success "环境配置完成"
}

# 交互式配置
interactive_config() {
    echo
    log_info "竞赛信息配置"
    echo "============="
    
    read -p "请输入团队名称: " TEAM_NAME
    read -p "请输入竞赛日期 (YYYY-MM-DD): " COMPETITION_DATE
    
    echo
    echo "请选择硬件配置:"
    echo "1) 标准配置 (Jetson Xavier NX + USB相机)"
    echo "2) 高性能配置 (Jetson AGX Xavier + 工业相机)"
    echo "3) 仿真配置 (笔记本电脑 + 仿真器)"
    read -p "选择配置 (1-3): " hw_choice
    
    case $hw_choice in
        1) HARDWARE_CONFIG="standard" ;;
        2) HARDWARE_CONFIG="high_performance" ;;
        3) HARDWARE_CONFIG="simulation" ;;
        *) log_error "无效选择"; exit 1 ;;
    esac
    
    log_info "配置信息:"
    log_info "  团队: $TEAM_NAME"
    log_info "  日期: $COMPETITION_DATE"
    log_info "  硬件: $HARDWARE_CONFIG"
}

# 数据准备检查
check_training_data() {
    log_info "检查训练数据..."
    
    DATA_DIR="training_data"
    if [ ! -d "$DATA_DIR" ]; then
        log_warning "训练数据目录不存在，创建示例结构..."
        create_data_structure
        return 1
    fi
    
    # 检查数据完整性
    required_classes=("square_frame" "circle_frame" "red_flag" "blue_flag" "green_flag")
    missing_data=false
    
    for class_name in "${required_classes[@]}"; do
        class_count=$(find $DATA_DIR -name "*${class_name}*" | wc -l)
        if [ $class_count -lt 100 ]; then
            log_warning "类别 $class_name 数据不足: $class_count 张 (建议 >1500 张)"
            missing_data=true
        else
            log_success "类别 $class_name 数据充足: $class_count 张"
        fi
    done
    
    if [ "$missing_data" = true ]; then
        echo
        log_warning "数据收集建议:"
        echo "1. 使用 scripts/collect_data.py 收集数据"
        echo "2. 确保各类数据平衡"
        echo "3. 覆盖不同光照和角度"
        return 1
    fi
    
    return 0
}

# 创建数据目录结构
create_data_structure() {
    mkdir -p training_data/{train,val,test}/{images,labels}
    
    cat > training_data/README.md << EOF
# 训练数据组织结构

## 目录说明
- train/: 训练数据 (80%)
- val/: 验证数据 (15%)  
- test/: 测试数据 (5%)

## 标注格式
使用YOLO格式标注:
- 图片: JPG/PNG格式
- 标签: TXT格式，每行一个目标
- 格式: class_id x_center y_center width height (相对坐标)

## 类别映射
0: square_frame  - 方形框架
1: circle_frame  - 圆形框架
2: red_flag      - 红色刀旗
3: blue_flag     - 蓝色刀旗
4: green_flag    - 绿色刀旗

## 数据收集要求
- 每类至少1500张标注图片
- 覆盖不同距离: 1-10米
- 覆盖不同角度: 360度
- 覆盖不同光照: 室内/室外/阴天/晴天
EOF
    
    log_success "数据目录结构已创建"
}

# YOLO模型训练
train_yolo_model() {
    log_info "开始训练YOLO模型..."
    
    if ! check_training_data; then
        log_error "训练数据检查失败，请先收集数据"
        return 1
    fi
    
    # 复制训练配置
    cp scripts/yolo_training_config.yaml training_data/
    
    # 更新配置文件中的路径
    sed -i "s|/path/to/competition_dataset|$WORKSPACE_ROOT/training_data|g" training_data/yolo_training_config.yaml
    
    # 开始训练
    python3 scripts/train_yolo.py \
        --config training_data/yolo_training_config.yaml \
        --model yolov8s \
        --export \
        --benchmark
        
    if [ $? -eq 0 ]; then
        log_success "YOLO模型训练完成"
        
        # 复制最佳模型到检测模块
        BEST_MODEL=$(find training_outputs -name "best.pt" | head -1)
        if [ -f "$BEST_MODEL" ]; then
            cp "$BEST_MODEL" src/object_det/scripts/Model/competition.pt
            log_success "模型已部署到检测模块"
        fi
    else
        log_error "YOLO模型训练失败"
        return 1
    fi
}

# 硬件配置
configure_hardware() {
    log_info "配置硬件参数..."
    
    case $HARDWARE_CONFIG in
        "standard")
            configure_standard_hardware
            ;;
        "high_performance")
            configure_high_performance_hardware
            ;;
        "simulation")
            configure_simulation_hardware
            ;;
    esac
}

configure_standard_hardware() {
    log_info "配置标准硬件..."
    
    # 更新相机参数
    cat > src/mission_pkg/config/camera_params.yaml << EOF
camera:
  device_id: 0
  width: 640
  height: 480
  fps: 30
  topic: "/camera/image_raw"
  
detection:
  model_path: "src/object_det/scripts/Model/competition.pt"
  confidence_threshold: 0.7
  device: "cpu"  # Jetson Xavier NX推荐CPU推理
EOF
    
    log_success "标准硬件配置完成"
}

configure_high_performance_hardware() {
    log_info "配置高性能硬件..."
    
    cat > src/mission_pkg/config/camera_params.yaml << EOF
camera:
  device_id: 0
  width: 1280
  height: 720
  fps: 60
  topic: "/camera/image_raw"
  
detection:
  model_path: "src/object_det/scripts/Model/competition.pt"
  confidence_threshold: 0.75
  device: "0"  # AGX Xavier使用GPU推理
EOF
    
    log_success "高性能硬件配置完成"
}

configure_simulation_hardware() {
    log_info "配置仿真环境..."
    
    cat > src/mission_pkg/config/camera_params.yaml << EOF
camera:
  device_id: 0
  width: 640
  height: 480
  fps: 30
  topic: "/camera/image_raw"
  
detection:
  model_path: "src/object_det/scripts/Model/competition.pt"
  confidence_threshold: 0.7
  device: "cpu"
EOF
    
    log_success "仿真环境配置完成"
}

# 系统测试
run_system_tests() {
    log_info "运行系统测试..."
    
    # 运行验证脚本
    ./validate_system.sh
    if [ $? -ne 0 ]; then
        log_error "系统验证失败"
        return 1
    fi
    
    # 测试YOLO检测
    log_info "测试目标检测..."
    timeout 30 python3 src/object_det/scripts/det.py --test || {
        log_warning "检测测试超时或失败"
    }
    
    # 测试行为树
    log_info "测试行为树..."
    timeout 60 roslaunch mission_pkg test.launch || {
        log_warning "行为树测试超时或失败" 
    }
    
    log_success "系统测试完成"
}

# 创建部署包
create_deployment_package() {
    log_info "创建部署包..."
    
    PACKAGE_NAME="competition_deployment_${TEAM_NAME}_${COMPETITION_DATE}"
    PACKAGE_DIR="deployment_packages/$PACKAGE_NAME"
    
    mkdir -p "$PACKAGE_DIR"
    
    # 复制核心文件
    cp -r src/mission_pkg "$PACKAGE_DIR/"
    cp -r src/object_det "$PACKAGE_DIR/"
    cp -r scripts "$PACKAGE_DIR/"
    cp validate_system.sh "$PACKAGE_DIR/"
    cp COMPETITION_DEPLOYMENT_MANUAL.md "$PACKAGE_DIR/"
    
    # 复制配置文件
    mkdir -p "$PACKAGE_DIR/backup_configs"
    cp -r src/mission_pkg/config "$PACKAGE_DIR/backup_configs/"
    
    # 创建部署说明
    cat > "$PACKAGE_DIR/DEPLOYMENT_README.md" << EOF
# 竞赛部署包

## 团队信息
- 团队名称: $TEAM_NAME
- 竞赛日期: $COMPETITION_DATE  
- 硬件配置: $HARDWARE_CONFIG
- 创建时间: $(date)

## 快速部署
1. 将此目录复制到目标系统
2. 运行: ./validate_system.sh
3. 启动: roslaunch mission_pkg competition.launch

## 文件说明
- mission_pkg/: 任务执行包
- object_det/: 目标检测包
- scripts/: 工具脚本
- backup_configs/: 备份配置
- COMPETITION_DEPLOYMENT_MANUAL.md: 完整部署手册

## 注意事项
- 确保ROS环境已配置
- 检查相机和传感器连接
- 验证模型文件完整性
- 在实际飞行前充分测试
EOF
    
    # 打包
    tar -czf "${PACKAGE_NAME}.tar.gz" -C deployment_packages "$PACKAGE_NAME"
    
    log_success "部署包已创建: ${PACKAGE_NAME}.tar.gz"
}

# 生成检查清单
generate_checklist() {
    log_info "生成竞赛检查清单..."
    
    cat > "competition_checklist_${COMPETITION_DATE}.md" << EOF
# 竞赛日检查清单

## 出发前检查 (T-24小时)
- [ ] 设备清单核对完成
- [ ] 软件系统测试通过  
- [ ] 备用配置准备完成
- [ ] 工具包装备齐全
- [ ] 团队分工明确

## 到场检查 (T-60分钟)
- [ ] 硬件设备完好
- [ ] 软件系统启动正常
- [ ] 通信链路测试通过
- [ ] 相机图像清晰
- [ ] GPS信号良好

## 场地标定 (T-45分钟)  
- [ ] 测量障碍物位置
- [ ] 更新配置参数
- [ ] 验证坐标系统
- [ ] 确认飞行边界

## 预飞测试 (T-30分钟)
- [ ] 手动起飞测试
- [ ] 悬停稳定性测试
- [ ] 目标检测测试
- [ ] 基本机动测试
- [ ] 安全降落测试

## 比赛执行 (T-10分钟)
- [ ] 电池电量检查
- [ ] 系统状态确认
- [ ] 应急预案就绪
- [ ] 通信设备检查
- [ ] 启动竞赛任务

## 应急预案
- [ ] 手动接管程序
- [ ] 通信中断处理
- [ ] 系统故障恢复
- [ ] 紧急降落程序

---
团队: $TEAM_NAME
日期: $COMPETITION_DATE
生成时间: $(date)
EOF
    
    log_success "检查清单已生成"
}

# 主函数
main() {
    echo
    log_info "开始竞赛系统部署流程..."
    
    # 交互式配置
    interactive_config
    
    # 系统检查和初始化
    check_dependencies
    initialize_system
    
    # 硬件配置
    configure_hardware
    
    # YOLO模型训练 (可选)
    echo
    read -p "是否需要训练YOLO模型? (y/N): " train_model
    if [[ $train_model =~ ^[Yy]$ ]]; then
        train_yolo_model
    else
        log_info "跳过模型训练，使用现有模型"
    fi
    
    # 系统测试
    run_system_tests
    
    # 创建部署包
    create_deployment_package
    
    # 生成检查清单
    generate_checklist
    
    echo
    log_success "部署流程完成！"
    echo
    echo "后续步骤:"
    echo "1. 查看部署手册: COMPETITION_DEPLOYMENT_MANUAL.md"
    echo "2. 检查清单: competition_checklist_${COMPETITION_DATE}.md"
    echo "3. 部署包: ${PACKAGE_NAME}.tar.gz"
    echo "4. 日志文件: $LOG_FILE"
    echo
    echo "祝比赛顺利！🚁"
}

# 脚本入口
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi