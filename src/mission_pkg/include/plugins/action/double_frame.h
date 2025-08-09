/******************************************************************************
 * @file       double_frame.h  
 * @brief      行为树：双框穿越节点 - 完成左进右出/右进左出动作
 * @author     AI Assistant
 * @date       2025/01/01
 * @history    为竞赛obs3障碍设计的双框穿越控制
 *****************************************************************************/

#ifndef DOUBLE_FRAME_H
#define DOUBLE_FRAME_H

#include "plugins/common.hpp"
#include <ros/ros.h>
#include <mavros_cnt.h>
#include <std_msgs/String.h>

using namespace BT;

class DoubleFrame : public StatefulActionNode
{
public:
  DoubleFrame(const std::string &name, const NodeConfig &config);

  // 定义参数变量获取
  static PortsList providedPorts();

  // 状态机方法
  NodeStatus onStart() override;
  NodeStatus onRunning() override;
  void onHalted() override;

private:
  // 双框穿越状态枚举
  enum DoubleFrameState {
    DETECTING_FRAMES,
    APPROACHING_ENTRY,
    PASSING_FIRST_FRAME,
    NAVIGATING_BETWEEN,
    PASSING_SECOND_FRAME,
    COMPLETED
  };

  std::shared_ptr<MavRosConnect> mct;
  const geometry_msgs::PoseStamped *fcu_pose_ptr;
  const mavros_msgs::State *fcu_state_ptr;
  std::shared_ptr<ros::Publisher> tgt_pose_pub_ptr;

  // 双框参数
  DoubleFrameState current_state;
  std::string traversal_direction; // "left_right" or "right_left"
  BT::Position3D first_frame_pos;
  BT::Position3D second_frame_pos;
  BT::Position3D entry_point;
  BT::Position3D intermediate_point;
  BT::Position3D exit_point;
  double approach_speed;
  ros::Time start_time;
  ros::Time state_start_time;

  // 控制参数
  mavros_msgs::PositionTarget cmd;
  
  // 检测相关
  ros::Subscriber detection_sub;
  bool frames_detected;
  
  // 辅助方法
  void detectionCallback(const std_msgs::String::ConstPtr& msg);
  bool isAtPosition(const BT::Position3D& target, double tolerance = 0.3);
  void calculateTrajectoryPoints();
  void publishCommand(const BT::Position3D& pos, const BT::Position3D& vel = BT::Position3D(0,0,0));
  BT::Position3D getCurrentPosition();
};

#endif // DOUBLE_FRAME_H