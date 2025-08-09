#include "plugins/action/somersault.h"
#include <cmath>

Somersault::Somersault(const std::string &name, const NodeConfig &config)
    : StatefulActionNode(name, config), current_state(APPROACHING_OBSTACLE)
{
  mct = MavRosConnect::getInstance();
  fcu_pose_ptr = mct->getFcuPose();
  fcu_state_ptr = mct->getFcuState();
  tgt_pose_pub_ptr = std::make_shared<ros::Publisher>(*mct->getPosTgtPublier());

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
}

PortsList Somersault::providedPorts()
{
  return {
    InputPort<Position3D>("entry_pos", "下框入口位置"),
    InputPort<Position3D>("exit_pos", "上框出口位置"),
    InputPort<double>("loop_radius", "翻滚半径", 1.0),
    InputPort<double>("loop_speed", "翻滚速度", 2.0)
  };
}

NodeStatus Somersault::onStart()
{
  ROS_INFO("Somersault: Starting somersault maneuver");
  
  // 获取参数
  auto entry_pos = getInput<Position3D>("entry_pos");
  auto exit_pos = getInput<Position3D>("exit_pos");
  auto radius = getInput<double>("loop_radius");
  auto speed = getInput<double>("loop_speed");

  if (!entry_pos || !exit_pos) {
    ROS_ERROR("Somersault: Missing required positions");
    return NodeStatus::FAILURE;
  }

  entry_point = entry_pos.value();
  exit_point = exit_pos.value();
  loop_radius = radius.value_or(1.0);
  loop_speed = speed.value_or(2.0);

  current_state = APPROACHING_OBSTACLE;
  start_time = ros::Time::now();
  state_start_time = start_time;

  ROS_INFO("Somersault: Entry(%.2f,%.2f,%.2f) Exit(%.2f,%.2f,%.2f) Radius:%.2f", 
           entry_point.x, entry_point.y, entry_point.z,
           exit_point.x, exit_point.y, exit_point.z, loop_radius);

  return NodeStatus::RUNNING;
}

NodeStatus Somersault::onRunning()
{
  ros::Time current_time = ros::Time::now();
  double state_duration = (current_time - state_start_time).toSec();

  switch (current_state) {
    case APPROACHING_OBSTACLE: {
      // 接近下框入口
      publishCommand(entry_point);
      
      if (isAtPosition(entry_point, 0.3)) {
        ROS_INFO("Somersault: Reached entry point, entering lower frame");
        current_state = ENTERING_LOWER_FRAME;
        state_start_time = current_time;
      }
      
      // 超时保护
      if (state_duration > 10.0) {
        ROS_WARN("Somersault: Timeout approaching obstacle");
        return NodeStatus::FAILURE;
      }
      break;
    }

    case ENTERING_LOWER_FRAME: {
      // 进入下框
      BT::Position3D through_point = entry_point;
      through_point.x += 0.5; // 向前穿过下框
      publishCommand(through_point);
      
      if (isAtPosition(through_point, 0.2)) {
        ROS_INFO("Somersault: Entered lower frame, starting loop maneuver");
        current_state = PERFORMING_LOOP;
        state_start_time = current_time;
      }
      
      if (state_duration > 5.0) {
        ROS_WARN("Somersault: Timeout entering lower frame");
        return NodeStatus::FAILURE;
      }
      break;
    }

    case PERFORMING_LOOP: {
      // 执行翻滚动作
      double loop_duration = 2.0 * M_PI * loop_radius / loop_speed;
      double phase = (state_duration / loop_duration) * 2.0 * M_PI;
      
      if (phase < 2.0 * M_PI) {
        BT::Position3D loop_pos, loop_vel;
        generateLoopTrajectory(phase, loop_pos, loop_vel);
        publishCommand(loop_pos, loop_vel);
      } else {
        ROS_INFO("Somersault: Loop completed, exiting through upper frame");
        current_state = EXITING_UPPER_FRAME;
        state_start_time = current_time;
      }
      break;
    }

    case EXITING_UPPER_FRAME: {
      // 从上框出口离开
      publishCommand(exit_point);
      
      if (isAtPosition(exit_point, 0.3)) {
        ROS_INFO("Somersault: Successfully completed somersault maneuver");
        current_state = COMPLETED;
        return NodeStatus::SUCCESS;
      }
      
      if (state_duration > 10.0) {
        ROS_WARN("Somersault: Timeout exiting upper frame");
        return NodeStatus::FAILURE;
      }
      break;
    }

    case COMPLETED:
      return NodeStatus::SUCCESS;
  }

  return NodeStatus::RUNNING;
}

void Somersault::onHalted()
{
  ROS_INFO("Somersault: Maneuver halted");
  current_state = APPROACHING_OBSTACLE;
}

bool Somersault::isAtPosition(const BT::Position3D& target, double tolerance)
{
  if (!fcu_pose_ptr) return false;
  
  double dx = fcu_pose_ptr->pose.position.x - target.x;
  double dy = fcu_pose_ptr->pose.position.y - target.y;
  double dz = fcu_pose_ptr->pose.position.z - target.z;
  
  return (sqrt(dx*dx + dy*dy + dz*dz) < tolerance);
}

void Somersault::generateLoopTrajectory(double phase, BT::Position3D& position, BT::Position3D& velocity)
{
  // 计算翻滚轨迹 - 垂直平面内的圆形轨迹
  // 以entry_point为起点，在x-z平面内做垂直翻滚
  
  double center_x = entry_point.x + loop_radius;
  double center_z = entry_point.z + loop_radius;
  
  // 圆形轨迹
  position.x = center_x + loop_radius * cos(phase - M_PI/2);
  position.y = entry_point.y;
  position.z = center_z + loop_radius * sin(phase - M_PI/2);
  
  // 速度计算
  velocity.x = -loop_radius * loop_speed / loop_radius * sin(phase - M_PI/2);
  velocity.y = 0;
  velocity.z = loop_radius * loop_speed / loop_radius * cos(phase - M_PI/2);
}

void Somersault::publishCommand(const BT::Position3D& pos, const BT::Position3D& vel)
{
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "somersault";
  
  cmd.position.x = pos.x;
  cmd.position.y = pos.y;
  cmd.position.z = pos.z;
  
  cmd.velocity.x = vel.x;
  cmd.velocity.y = vel.y;
  cmd.velocity.z = vel.z;
  
  cmd.yaw = 0; // 保持朝向
  
  tgt_pose_pub_ptr->publish(cmd);
}