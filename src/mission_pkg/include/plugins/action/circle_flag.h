/******************************************************************************
 * @file       circle_flag.h
 * @brief      行为树：环绕刀旗节点 - 完成环绕指定颜色刀旗动作
 * @author     AI Assistant
 * @date       2025/01/01
 * @history    为竞赛obs5障碍设计的环绕控制
 *****************************************************************************/

#ifndef CIRCLE_FLAG_H
#define CIRCLE_FLAG_H

#include "plugins/common.hpp"
#include <ros/ros.h>
#include "mavros_cnt.h"
#include <std_msgs/String.h>

using namespace BT;

class CircleFlag : public StatefulActionNode
{
public:
  CircleFlag(const std::string &name, const NodeConfig &config);

  // 定义参数变量获取
  static PortsList providedPorts();

  // 状态机方法
  NodeStatus onStart() override;
  NodeStatus onRunning() override;
  void onHalted() override;

private:
  // 环绕状态枚举
  enum CircleState {
    DETECTING_FLAG,
    APPROACHING_CIRCLE,
    CIRCLING,
    COMPLETED
  };

  std::shared_ptr<MavRosConnect> mct;
  const geometry_msgs::PoseStamped *fcu_pose_ptr;
  const mavros_msgs::State *fcu_state_ptr;
  std::shared_ptr<ros::Publisher> tgt_pose_pub_ptr;

  // 环绕参数
  CircleState current_state;
  std::string target_flag_color;
  BT::Position3D flag_position;
  double circle_radius;
  double circle_speed;
  double circle_height;
  ros::Time start_time;
  ros::Time circle_start_time;
  double total_angle_covered;
  
  // 控制参数
  mavros_msgs::PositionTarget cmd;
  
  // 检测相关
  ros::Subscriber detection_sub;
  bool flag_detected;
  
  // 辅助方法
  void detectionCallback(const std_msgs::String::ConstPtr& msg);
  bool isAtPosition(const BT::Position3D& target, double tolerance = 0.3);
  void generateCircleTrajectory(double angle, BT::Position3D& position, BT::Position3D& velocity);
  void publishCommand(const BT::Position3D& pos, const BT::Position3D& vel = BT::Position3D(0,0,0));
  BT::Position3D getCurrentPosition();
};

#endif // CIRCLE_FLAG_H