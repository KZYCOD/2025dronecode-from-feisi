/******************************************************************************
 * @file       plannode.h
 * @brief      给规划器发送目标点，然后接受规划器的指令，并下发给飞控
 *
 * @author     戴开世<daivesi@sina.com>
 * @date       2025/03/20
 * @history
 *****************************************************************************/

#ifndef PLANNODE_H
#define PLANNODE_H
#include <plugins/common.hpp>
#include <mavros_cnt.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

using namespace BT;

class PlanNodeRviz : public StatefulActionNode
{
public:
  PlanNodeRviz(const std::string name, const NodeConfiguration &config);

  static PortsList providedPorts();

  //订阅估计规划的飞行命令，同时需要设置一个开关，可以屏蔽以及开启随机给跟随
  void PositionCmdCB(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
  void GoalRecv(const geometry_msgs::PoseStamped::ConstPtr &msg);

  NodeStatus onStart() override;
  NodeStatus onRunning() override;
  void       onHalted() override;

private:
  std::shared_ptr<MavRosConnect>    mct;
  std::shared_ptr<ros::NodeHandle>  nh;
  const geometry_msgs::PoseStamped *fcu_pose_ptr;
  const mavros_msgs::State *        fcu_state_ptr;
  const ros::Publisher *            tgt_pose_pub_ptr;
  ros::Subscriber                   planer_goal_recv;
  ros::Publisher                    planner_goal_pub;
  ros::Subscriber                   pos_cmd_sub;
  mavros_msgs::PositionTarget       cmd;
  bool                              is_enable_planner;
  bool                              has_recv_cmd;
  ros::Time                         last_time;
  geometry_msgs::PoseStamped        goal;
};

#endif  // PLANNODE_H
