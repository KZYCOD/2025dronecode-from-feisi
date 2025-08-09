#include "plugins/action/double_frame.h"
#include <cmath>
#include <std_msgs/String.h>
#include <string>

DoubleFrame::DoubleFrame(const std::string &name, const NodeConfig &config)
    : StatefulActionNode(name, config), current_state(DETECTING_FRAMES), frames_detected(false)
{
  mct = MavRosConnect::getInstance();
  fcu_pose_ptr = mct->getFcuPose();
  fcu_state_ptr = mct->getFcuState();
  tgt_pose_pub_ptr = mct->getPosTgtPublier();

  // 初始化控制命令
  cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  cmd.type_mask = 0xfff;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::FORCE;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PX;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PY;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PZ;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VX;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VY;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VZ;
  cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW;

  // 订阅检测结果
  auto nh = mct->getROSHandle();
  detection_sub = nh->subscribe("/object_detection/frame_detection", 10, 
                               &DoubleFrame::detectionCallback, this);
}

PortsList DoubleFrame::providedPorts()
{
  return {
    InputPort<std::string>("direction", "穿越方向: left_right 或 right_left", "left_right"),
    InputPort<Position3D>("first_frame", "第一个框位置"),
    InputPort<Position3D>("second_frame", "第二个框位置"),
    InputPort<double>("approach_speed", "接近速度", 1.0)
  };
}

NodeStatus DoubleFrame::onStart()
{
  ROS_INFO("DoubleFrame: Starting double frame traversal maneuver");
  
  // 获取参数
  auto direction = getInput<std::string>("direction");
  auto frame1_pos = getInput<Position3D>("first_frame");
  auto frame2_pos = getInput<Position3D>("second_frame");
  auto speed = getInput<double>("approach_speed");

  traversal_direction = direction.value_or("left_right");
  approach_speed = speed.value_or(1.0);

  // 如果提供了框位置，直接使用；否则需要检测
  if (frame1_pos && frame2_pos) {
    first_frame_pos = frame1_pos.value();
    second_frame_pos = frame2_pos.value();
    frames_detected = true;
    
    calculateTrajectoryPoints();
    current_state = APPROACHING_ENTRY;
  } else {
    current_state = DETECTING_FRAMES;
    frames_detected = false;
  }

  start_time = ros::Time::now();
  state_start_time = start_time;

  ROS_INFO("DoubleFrame: Direction=%s, Speed=%.2f", 
           traversal_direction.c_str(), approach_speed);

  return NodeStatus::RUNNING;
}

NodeStatus DoubleFrame::onRunning()
{
  ros::Time current_time = ros::Time::now();
  double state_duration = (current_time - state_start_time).toSec();

  switch (current_state) {
    case DETECTING_FRAMES: {
      // 检测双框位置
      if (frames_detected) {
        ROS_INFO("DoubleFrame: Frames detected, calculating trajectory");
        calculateTrajectoryPoints();
        current_state = APPROACHING_ENTRY;
        state_start_time = current_time;
      }
      
      // 检测超时
      if (state_duration > 10.0) {
        ROS_WARN("DoubleFrame: Frame detection timeout");
        return NodeStatus::FAILURE;
      }
      break;
    }

    case APPROACHING_ENTRY: {
      // 接近入口点
      publishCommand(entry_point);
      
      if (isAtPosition(entry_point, 0.3)) {
        ROS_INFO("DoubleFrame: Reached entry point, passing first frame");
        current_state = PASSING_FIRST_FRAME;
        state_start_time = current_time;
      }
      
      if (state_duration > 10.0) {
        ROS_WARN("DoubleFrame: Timeout approaching entry");
        return NodeStatus::FAILURE;
      }
      break;
    }

    case PASSING_FIRST_FRAME: {
      // 穿过第一个框
      publishCommand(intermediate_point);
      
      if (isAtPosition(intermediate_point, 0.2)) {
        ROS_INFO("DoubleFrame: Passed first frame, navigating between frames");
        current_state = NAVIGATING_BETWEEN;
        state_start_time = current_time;
      }
      
      if (state_duration > 8.0) {
        ROS_WARN("DoubleFrame: Timeout passing first frame");
        return NodeStatus::FAILURE;
      }
      break;
    }

    case NAVIGATING_BETWEEN: {
      // 在两框之间导航到第二框入口
      BT::Position3D second_entry;
      if (traversal_direction == "left_right") {
        second_entry = second_frame_pos;
        second_entry.x -= 0.8; // 从左侧接近第二框
      } else {
        second_entry = second_frame_pos;
        second_entry.x += 0.8; // 从右侧接近第二框
      }
      
      publishCommand(second_entry);
      
      if (isAtPosition(second_entry, 0.3)) {
        ROS_INFO("DoubleFrame: Positioned for second frame, passing through");
        current_state = PASSING_SECOND_FRAME;
        state_start_time = current_time;
      }
      
      if (state_duration > 10.0) {
        ROS_WARN("DoubleFrame: Timeout navigating between frames");
        return NodeStatus::FAILURE;
      }
      break;
    }

    case PASSING_SECOND_FRAME: {
      // 穿过第二个框到达出口
      publishCommand(exit_point);
      
      if (isAtPosition(exit_point, 0.3)) {
        ROS_INFO("DoubleFrame: Successfully completed double frame traversal");
        current_state = COMPLETED;
        return NodeStatus::SUCCESS;
      }
      
      if (state_duration > 8.0) {
        ROS_WARN("DoubleFrame: Timeout passing second frame");
        return NodeStatus::FAILURE;
      }
      break;
    }

    case COMPLETED:
      return NodeStatus::SUCCESS;
  }

  return NodeStatus::RUNNING;
}

void DoubleFrame::onHalted()
{
  ROS_INFO("DoubleFrame: Maneuver halted");
  current_state = DETECTING_FRAMES;
  frames_detected = false;
}

void DoubleFrame::detectionCallback(const std_msgs::String::ConstPtr& msg)
{
  // 简化的检测回调 - 实际应该解析检测结果获取框的精确位置
  if (msg->data.find("double_frame") != std::string::npos && !frames_detected) {
    ROS_INFO("DoubleFrame: Double frames detected");
    
    // 这里应该从检测消息中解析框的实际位置
    // 现在使用默认位置作为示例
    BT::Position3D current_pos = getCurrentPosition();
    first_frame_pos = BT::Position3D(current_pos.x + 2, current_pos.y - 1, current_pos.z);
    second_frame_pos = BT::Position3D(current_pos.x + 2, current_pos.y + 1, current_pos.z);
    
    frames_detected = true;
  }
}

bool DoubleFrame::isAtPosition(const BT::Position3D& target, double tolerance)
{
  if (!fcu_pose_ptr) return false;
  
  double dx = fcu_pose_ptr->pose.position.x - target.x;
  double dy = fcu_pose_ptr->pose.position.y - target.y;
  double dz = fcu_pose_ptr->pose.position.z - target.z;
  
  return (sqrt(dx*dx + dy*dy + dz*dz) < tolerance);
}

void DoubleFrame::calculateTrajectoryPoints()
{
  // 根据穿越方向计算轨迹点
  if (traversal_direction == "left_right") {
    // 左进右出
    entry_point = first_frame_pos;
    entry_point.x -= 1.0; // 从左侧接近第一框
    
    intermediate_point = first_frame_pos;
    intermediate_point.x += 0.5; // 穿过第一框
    
    exit_point = second_frame_pos;
    exit_point.x += 1.0; // 从第二框右侧离开
  } else {
    // 右进左出
    entry_point = first_frame_pos;
    entry_point.x += 1.0; // 从右侧接近第一框
    
    intermediate_point = first_frame_pos;
    intermediate_point.x -= 0.5; // 穿过第一框
    
    exit_point = second_frame_pos;
    exit_point.x -= 1.0; // 从第二框左侧离开
  }
  
  ROS_INFO("DoubleFrame: Trajectory calculated - Entry(%.2f,%.2f,%.2f) Exit(%.2f,%.2f,%.2f)",
           entry_point.x, entry_point.y, entry_point.z,
           exit_point.x, exit_point.y, exit_point.z);
}

void DoubleFrame::publishCommand(const BT::Position3D& pos, const BT::Position3D& vel)
{
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "double_frame";
  
  cmd.position.x = pos.x;
  cmd.position.y = pos.y;
  cmd.position.z = pos.z;
  
  cmd.velocity.x = vel.x;
  cmd.velocity.y = vel.y;
  cmd.velocity.z = vel.z;
  
  cmd.yaw = 0; // 保持朝向
  
  tgt_pose_pub_ptr->publish(cmd);
}

BT::Position3D DoubleFrame::getCurrentPosition()
{
  if (!fcu_pose_ptr) return BT::Position3D(0, 0, 0);
  
  return BT::Position3D(
    fcu_pose_ptr->pose.position.x,
    fcu_pose_ptr->pose.position.y,
    fcu_pose_ptr->pose.position.z
  );
}