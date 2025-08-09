#include "plugins/action/circle_flag.h"
#include <cmath>
#include <std_msgs/String.h>
#include <string>

CircleFlag::CircleFlag(const std::string &name, const NodeConfig &config)
    : StatefulActionNode(name, config), current_state(DETECTING_FLAG), 
      flag_detected(false), total_angle_covered(0.0)
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
  detection_sub = nh->subscribe("/object_detection/flag_detection", 10, 
                               &CircleFlag::detectionCallback, this);
}

PortsList CircleFlag::providedPorts()
{
  return {
    InputPort<std::string>("flag_color", "目标刀旗颜色"),
    InputPort<Position3D>("search_area", "搜索区域中心", Position3D(5, 5, 2)),
    InputPort<double>("circle_radius", "环绕半径", 2.0),
    InputPort<double>("circle_speed", "环绕速度", 1.5),
    InputPort<double>("circle_height", "环绕高度", 2.0)
  };
}

NodeStatus CircleFlag::onStart()
{
  ROS_INFO("CircleFlag: Starting flag circling maneuver");
  
  // 获取参数
  auto flag_color = getInput<std::string>("flag_color");
  auto search_pos = getInput<Position3D>("search_area");
  auto radius = getInput<double>("circle_radius");
  auto speed = getInput<double>("circle_speed");
  auto height = getInput<double>("circle_height");

  if (!flag_color) {
    ROS_ERROR("CircleFlag: Missing flag color parameter");
    return NodeStatus::FAILURE;
  }

  target_flag_color = flag_color.value();
  circle_radius = radius.value_or(2.0);
  circle_speed = speed.value_or(1.5);
  circle_height = height.value_or(2.0);

  // 设置搜索起始位置
  if (search_pos) {
    flag_position = search_pos.value();
  } else {
    flag_position = Position3D(5, 5, 2); // 默认搜索位置
  }

  current_state = DETECTING_FLAG;
  start_time = ros::Time::now();
  flag_detected = false;
  total_angle_covered = 0.0;

  ROS_INFO("CircleFlag: Searching for %s flag, radius:%.2f, speed:%.2f", 
           target_flag_color.c_str(), circle_radius, circle_speed);

  return NodeStatus::RUNNING;
}

NodeStatus CircleFlag::onRunning()
{
  ros::Time current_time = ros::Time::now();
  double state_duration = (current_time - start_time).toSec();

  switch (current_state) {
    case DETECTING_FLAG: {
      // 飞行到搜索区域并检测刀旗
      publishCommand(flag_position);
      
      if (flag_detected) {
        ROS_INFO("CircleFlag: Flag detected, approaching circle position");
        current_state = APPROACHING_CIRCLE;
        start_time = current_time;
      }
      
      // 搜索超时
      if (state_duration > 15.0) {
        ROS_WARN("CircleFlag: Flag detection timeout");
        return NodeStatus::FAILURE;
      }
      break;
    }

    case APPROACHING_CIRCLE: {
      // 计算环绕起始点
      BT::Position3D circle_start = flag_position;
      circle_start.x += circle_radius;
      circle_start.z = circle_height;
      
      publishCommand(circle_start);
      
      if (isAtPosition(circle_start, 0.3)) {
        ROS_INFO("CircleFlag: Starting circle maneuver around flag");
        current_state = CIRCLING;
        circle_start_time = current_time;
        total_angle_covered = 0.0;
      }
      
      if (state_duration > 10.0) {
        ROS_WARN("CircleFlag: Timeout approaching circle position");
        return NodeStatus::FAILURE;
      }
      break;
    }

    case CIRCLING: {
      // 执行环绕动作
      double circle_duration = (current_time - circle_start_time).toSec();
      double angular_velocity = circle_speed / circle_radius;
      double current_angle = angular_velocity * circle_duration;
      
      BT::Position3D circle_pos, circle_vel;
      generateCircleTrajectory(current_angle, circle_pos, circle_vel);
      publishCommand(circle_pos, circle_vel);
      
      total_angle_covered = current_angle;
      
      // 完成一圈 (2π radians)
      if (total_angle_covered >= 2.0 * M_PI) {
        ROS_INFO("CircleFlag: Successfully completed circle around flag");
        current_state = COMPLETED;
        return NodeStatus::SUCCESS;
      }
      
      // 环绕超时保护
      if (circle_duration > 20.0) {
        ROS_WARN("CircleFlag: Circle maneuver timeout");
        return NodeStatus::FAILURE;
      }
      break;
    }

    case COMPLETED:
      return NodeStatus::SUCCESS;
  }

  return NodeStatus::RUNNING;
}

void CircleFlag::onHalted()
{
  ROS_INFO("CircleFlag: Maneuver halted");
  current_state = DETECTING_FLAG;
  flag_detected = false;
  total_angle_covered = 0.0;
}

void CircleFlag::detectionCallback(const std_msgs::String::ConstPtr& msg)
{
  // 简化的检测回调 - 实际应该解析检测结果
  if (msg->data.find(target_flag_color) != std::string::npos) {
    if (!flag_detected) {
      ROS_INFO("CircleFlag: Target flag '%s' detected", target_flag_color.c_str());
      flag_detected = true;
      
      // 这里应该从检测消息中解析旗子的实际位置
      // 现在使用当前位置作为近似
      BT::Position3D current_pos = getCurrentPosition();
      flag_position = current_pos;
    }
  }
}

bool CircleFlag::isAtPosition(const BT::Position3D& target, double tolerance)
{
  if (!fcu_pose_ptr) return false;
  
  double dx = fcu_pose_ptr->pose.position.x - target.x;
  double dy = fcu_pose_ptr->pose.position.y - target.y;
  double dz = fcu_pose_ptr->pose.position.z - target.z;
  
  return (sqrt(dx*dx + dy*dy + dz*dz) < tolerance);
}

void CircleFlag::generateCircleTrajectory(double angle, BT::Position3D& position, BT::Position3D& velocity)
{
  // 在水平面内围绕旗子做圆周运动
  position.x = flag_position.x + circle_radius * cos(angle);
  position.y = flag_position.y + circle_radius * sin(angle);
  position.z = circle_height;
  
  // 切向速度
  double angular_velocity = circle_speed / circle_radius;
  velocity.x = -circle_radius * angular_velocity * sin(angle);
  velocity.y = circle_radius * angular_velocity * cos(angle);
  velocity.z = 0;
}

void CircleFlag::publishCommand(const BT::Position3D& pos, const BT::Position3D& vel)
{
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "circle_flag";
  
  cmd.position.x = pos.x;
  cmd.position.y = pos.y;
  cmd.position.z = pos.z;
  
  cmd.velocity.x = vel.x;
  cmd.velocity.y = vel.y;
  cmd.velocity.z = vel.z;
  
  // 计算朝向旗子的偏航角
  double yaw = atan2(flag_position.y - pos.y, flag_position.x - pos.x);
  cmd.yaw = yaw;
  
  tgt_pose_pub_ptr->publish(cmd);
}

BT::Position3D CircleFlag::getCurrentPosition()
{
  if (!fcu_pose_ptr) return BT::Position3D(0, 0, 0);
  
  return BT::Position3D(
    fcu_pose_ptr->pose.position.x,
    fcu_pose_ptr->pose.position.y,
    fcu_pose_ptr->pose.position.z
  );
}