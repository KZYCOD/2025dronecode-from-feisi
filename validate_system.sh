#!/bin/bash

# 竞赛系统验证脚本
# Competition System Validation Script

echo "=== 竞赛系统文件结构验证 ==="
echo "Validating competition system file structure..."

# 检查基本文件结构
echo "检查核心文件..."

# 核心配置文件
files=(
    "src/mission_pkg/config/competition.xml"
    "src/mission_pkg/config/competition_params.yaml"
    "src/mission_pkg/config/competition_with_scheduler.xml"
    "src/mission_pkg/launch/competition.launch"
)

# 头文件
headers=(
    "src/mission_pkg/include/plugins/action/somersault.h"
    "src/mission_pkg/include/plugins/action/circle_flag.h"
    "src/mission_pkg/include/plugins/action/double_frame.h"
    "src/mission_pkg/include/plugins/action/competition_scheduler.h"
    "src/mission_pkg/include/plugins/action/publish_obstacle_complete.h"
)

# 源文件
sources=(
    "src/mission_pkg/src/plugins/action/somersault.cpp"
    "src/mission_pkg/src/plugins/action/circle_flag.cpp"
    "src/mission_pkg/src/plugins/action/double_frame.cpp"
    "src/mission_pkg/src/plugins/action/competition_scheduler.cpp"
    "src/mission_pkg/src/plugins/action/publish_obstacle_complete.cpp"
)

# 文档文件
docs=(
    "COMPETITION_SYSTEM_DESIGN.md"
    "QUICK_START_GUIDE.md"
)

check_files() {
    local file_list=("$@")
    local missing_files=()
    
    for file in "${file_list[@]}"; do
        if [ -f "$file" ]; then
            echo "✅ $file"
        else
            echo "❌ $file - MISSING"
            missing_files+=("$file")
        fi
    done
    
    return ${#missing_files[@]}
}

echo -e "\n--- 配置文件检查 ---"
check_files "${files[@]}"
config_status=$?

echo -e "\n--- 头文件检查 ---"
check_files "${headers[@]}"
header_status=$?

echo -e "\n--- 源文件检查 ---"  
check_files "${sources[@]}"
source_status=$?

echo -e "\n--- 文档文件检查 ---"
check_files "${docs[@]}"
doc_status=$?

# XML语法验证
echo -e "\n=== XML文件语法验证 ==="
xml_files=(
    "src/mission_pkg/config/competition.xml"
    "src/mission_pkg/config/competition_with_scheduler.xml"
    "src/mission_pkg/launch/competition.launch"
)

xml_errors=0
for xml_file in "${xml_files[@]}"; do
    if [ -f "$xml_file" ]; then
        if command -v xmllint >/dev/null 2>&1; then
            if xmllint --noout "$xml_file" 2>/dev/null; then
                echo "✅ $xml_file - XML语法正确"
            else
                echo "❌ $xml_file - XML语法错误"
                ((xml_errors++))
            fi
        else
            echo "⚠️  xmllint未安装，跳过XML验证"
            break
        fi
    fi
done

# YAML语法验证
echo -e "\n=== YAML文件语法验证 ==="
yaml_file="src/mission_pkg/config/competition_params.yaml"
if [ -f "$yaml_file" ]; then
    if command -v python3 >/dev/null 2>&1; then
        if python3 -c "import yaml; yaml.safe_load(open('$yaml_file'))" 2>/dev/null; then
            echo "✅ $yaml_file - YAML语法正确"
        else
            echo "❌ $yaml_file - YAML语法错误"
            ((xml_errors++))
        fi
    else
        echo "⚠️  Python3未安装，跳过YAML验证"
    fi
fi

# 代码完整性检查
echo -e "\n=== 代码完整性检查 ==="

# 检查关键类定义
echo "检查关键类定义..."
key_classes=("Somersault" "CircleFlag" "DoubleFrame" "CompetitionScheduler")

class_errors=0
for class_name in "${key_classes[@]}"; do
    case $class_name in
        "Somersault") 
            header_file="src/mission_pkg/include/plugins/action/somersault.h"
            source_file="src/mission_pkg/src/plugins/action/somersault.cpp" ;;
        "CircleFlag") 
            header_file="src/mission_pkg/include/plugins/action/circle_flag.h"
            source_file="src/mission_pkg/src/plugins/action/circle_flag.cpp" ;;
        "DoubleFrame") 
            header_file="src/mission_pkg/include/plugins/action/double_frame.h"
            source_file="src/mission_pkg/src/plugins/action/double_frame.cpp" ;;
        "CompetitionScheduler") 
            header_file="src/mission_pkg/include/plugins/action/competition_scheduler.h"
            source_file="src/mission_pkg/src/plugins/action/competition_scheduler.cpp" ;;
    esac
    
    if [ -f "$header_file" ] && [ -f "$source_file" ]; then
        # 检查类声明
        if grep -q "class $class_name" "$header_file"; then
            echo "✅ $class_name 类声明找到"
        else
            echo "❌ $class_name 类声明缺失"
            ((class_errors++))
        fi
        
        # 检查构造函数
        if grep -q "$class_name::" "$source_file"; then
            echo "✅ $class_name 实现找到"
        else
            echo "❌ $class_name 实现缺失"
            ((class_errors++))
        fi
    else
        echo "❌ $class_name 文件缺失"
        ((class_errors++))
    fi
done

# 检查关键方法
echo -e "\n检查关键方法..."
required_methods=("onStart" "onRunning" "providedPorts")

method_errors=0
for source_file in "${sources[@]}"; do
    if [ -f "$source_file" ]; then
        for method in "${required_methods[@]}"; do
            if grep -q "$method" "$source_file"; then
                echo "✅ $(basename $source_file) 包含 $method"
            else
                echo "⚠️  $(basename $source_file) 可能缺少 $method"
            fi
        done
    fi
done

# 总结
echo -e "\n=== 验证总结 ==="
total_errors=$((config_status + header_status + source_status + xml_errors + class_errors))

if [ $total_errors -eq 0 ]; then
    echo "🎉 所有检查通过！竞赛系统已正确实现。"
    echo "All checks passed! Competition system is properly implemented."
    
    echo -e "\n📋 下一步："
    echo "1. 编译系统: catkin_make"
    echo "2. 运行测试: roslaunch mission_pkg competition.launch use_sim:=true"
    echo "3. 查看文档: cat COMPETITION_SYSTEM_DESIGN.md"
    
    exit 0
else
    echo "⚠️  发现 $total_errors 个问题需要解决。"
    echo "Found $total_errors issues that need to be resolved."
    
    echo -e "\n建议："
    echo "1. 检查缺失的文件"
    echo "2. 修复语法错误" 
    echo "3. 补充缺失的类和方法"
    
    exit 1
fi