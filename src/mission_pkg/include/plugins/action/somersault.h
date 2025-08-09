/******************************************************************************
 * @file       somersault.h
 * @brief      行为树：翻滚动作节点 - 完成上下翻跟斗动作
 * @author     AI Assistant
 * @date       2025/01/01
 * @history    为竞赛obs4障碍设计的翻滚控制
 *****************************************************************************/

#ifndef SOMERSAULT_H
#define SOMERSAULT_H

#include "plugins/common.hpp"
#include <ros/ros.h>
#include <mavros_cnt.h>

using namespace BT;

class Somersault : public StatefulActionNode
{
public:
  Somersault(const std::string &name, const NodeConfig &config);

  // 定义参数变量获取
  static PortsList providedPorts();

  // 状态机方法
  NodeStatus onStart() override;
  NodeStatus onRunning() override;
  void onHalted() override;

private:
  // 翻滚状态枚举
  enum SomersaultState {
    APPROACHING_OBSTACLE,
    ENTERING_LOWER_FRAME,
    PERFORMING_LOOP,
    EXITING_UPPER_FRAME,
    COMPLETED
  };

  std::shared_ptr<MavRosConnect> mct;
  const geometry_msgs::PoseStamped *fcu_pose_ptr;
  const mavros_msgs::State *fcu_state_ptr;
  std::shared_ptr<ros::Publisher> tgt_pose_pub_ptr;

  // 翻滚参数
  SomersaultState current_state;
  BT::Position3D entry_point;
  BT::Position3D exit_point;
  double loop_radius;
  double loop_speed;
  ros::Time start_time;
  ros::Time state_start_time;

  // 控制参数
  mavros_msgs::PositionTarget cmd;
  
  // 辅助方法
  bool isAtPosition(const BT::Position3D& target, double tolerance = 0.3);
  void generateLoopTrajectory(double phase, BT::Position3D& position, BT::Position3D& velocity);
  void publishCommand(const BT::Position3D& pos, const BT::Position3D& vel = BT::Position3D(0,0,0));
};

#endif // SOMERSAULT_H