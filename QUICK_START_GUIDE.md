# 竞赛系统快速开始指南

## 快速启动

### 1. 准备环境
```bash
# 确保ROS环境已配置
source /opt/ros/noetic/setup.bash  # 或对应的ROS版本

# 编译工作空间
cd /path/to/your/catkin_workspace
catkin_make

# 设置环境变量
source devel/setup.bash
```

### 2. 启动仿真测试
```bash
# 启动基础竞赛任务
roslaunch mission_pkg competition.launch use_sim:=true debug_mode:=true

# 或启动带调度器的完整版本
roslaunch mission_pkg competition.launch config_path:="$(rospack find mission_pkg)/config/competition_with_scheduler.xml"
```

### 3. 监控任务状态
```bash
# 查看任务状态
rostopic echo /competition/state

# 查看任务进度
rostopic echo /competition/status

# 查看无人机位置
rostopic echo /mavros/local_position/pose
```

## 核心代码示例

### 使用翻滚控制
```cpp
// 在行为树XML中配置
<Somersault entry_pos="8.0;0.0;1.5" 
           exit_pos="8.0;0.0;2.5" 
           loop_radius="1.0" 
           loop_speed="2.0" />
```

### 使用环绕控制
```cpp
// 在行为树XML中配置
<CircleFlag flag_color="red" 
           search_area="9.0;0.0;2.0" 
           circle_radius="2.0" 
           circle_speed="1.5" />
```

## 参数调整示例

### 修改障碍位置
编辑 `config/competition_params.yaml`:
```yaml
obstacles:
  obs1_square_frame:
    position: [2.0, 0.0, 2.0]  # 调整方框位置
    width: 1.5
    height: 1.5
    
  obs4_somersault:
    lower_frame_position: [8.0, 0.0, 1.5]  # 调整翻滚起始高度
    upper_frame_position: [8.0, 0.0, 2.5]
    loop_radius: 1.2  # 调整翻滚半径
```

### 调整安全参数
```yaml
safety:
  emergency_stop_height: 0.5
  timeout_general: 45.0      # 增加超时时间
  timeout_maneuver: 30.0
```

## 故障排除

### 常见问题

1. **编译错误**
   ```bash
   # 检查依赖包
   rosdep install --from-paths src --ignore-src -r -y
   
   # 清理并重新编译
   catkin_make clean
   catkin_make
   ```

2. **MAVROS连接失败**
   ```bash
   # 检查飞控连接
   ls /dev/ttyACM*
   
   # 启动MAVROS
   roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600
   ```

3. **行为树解析错误**
   ```bash
   # 验证XML语法
   xmllint --noout config/competition.xml
   
   # 检查节点注册
   rosrun mission_pkg bt_ros is_gen_xml:=true xml_file:=nodes.xml
   ```

### 调试技巧

1. **启用详细日志**
   ```xml
   <node pkg="mission_pkg" name="competition_mission" type="bt_ros" output="screen">
     <param name="is_debug" value="true" type="bool" />
   </node>
   ```

2. **可视化状态**
   ```bash
   # 启动RViz
   rviz -d $(rospack find mission_pkg)/rviz/competition.rviz
   ```

3. **录制数据**
   ```bash
   # 记录关键话题
   rosbag record /mavros/local_position/pose /competition/state /competition/status
   ```

## 高级配置

### 自定义障碍序列
创建自己的行为树配置：
```xml
<BehaviorTree ID="CustomCompetition">
  <Sequence>
    <!-- 自定义起飞高度 -->
    <SetBlackboard value="0.0;0.0;3.0" output_key="takeoff_goal" />
    <SubTree ID="takeoff" goal="{takeoff_goal}"/>
    
    <!-- 仅执行特定障碍 -->
    <Somersault entry_pos="5.0;0.0;1.5" exit_pos="5.0;0.0;2.5" />
    
    <!-- 自定义降落 -->
    <SubTree ID="Land" />
  </Sequence>
</BehaviorTree>
```

### 集成外部传感器
在launch文件中添加传感器节点：
```xml
<!-- 激光雷达 -->
<include file="$(find livox_ros_driver2)/launch/rviz_MID360.launch" />

<!-- 相机 -->
<node pkg="usb_cam" type="usb_cam_node" name="camera">
  <param name="image_topic" value="/camera/image_raw" />
</node>
```

## 性能优化

### 1. 控制参数调优
```yaml
control:
  position_tolerance: 0.2    # 降低精度要求以提高速度
  velocity_max: 3.0         # 提高最大速度
  acceleration_max: 1.5     # 提高加速度
```

### 2. 检测算法优化
```yaml
vision:
  confidence_threshold: 0.6  # 降低检测阈值
  frame_rate: 20            # 降低处理频率
```

### 3. 路径规划优化
在ego-planner中调整参数以优化轨迹平滑度和计算速度。

## 实际飞行注意事项

1. **安全检查清单**
   - [ ] 确认场地边界设置正确
   - [ ] 验证障碍物位置测量准确  
   - [ ] 测试紧急停止功能
   - [ ] 检查电池电量和信号强度

2. **飞行前测试**
   ```bash
   # 仿真验证
   roslaunch mission_pkg competition.launch use_sim:=true
   
   # 硬件在环测试
   roslaunch mission_pkg competition.launch use_sim:=false debug_mode:=true
   ```

3. **实时监控**
   - 监控飞行状态和位置
   - 观察任务执行进度
   - 准备手动接管

通过以上配置和示例，您可以快速上手并根据具体需求调整竞赛系统。