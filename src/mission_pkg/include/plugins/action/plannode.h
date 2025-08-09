/******************************************************************************
 * @file       plannode.h
 * @brief      给规划器发送目标点，然后接受规划器的指令，并下发给飞控
 * 节点运行中，有退出规划节点，有暂停规划(即规划器有指令发出，但是节点可以同个这个状态不接受轨迹指令)
 * @author     戴开世<daivesi@sina.com>
 * @date       2025/03/20
 * @history
 *****************************************************************************/

#ifndef PLANNODE_H
#define PLANNODE_H
#include <plugins/common.hpp>
#include "mavros_cnt.h"
#include <quadrotor_msgs/PositionCommand.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

using namespace BT;

class PlanNode : public StatefulActionNode
{
public:
  PlanNode(const std::string name, const NodeConfiguration &config);

  static PortsList providedPorts();

  // 订阅估计规划的飞行命令，同时需要设置一个开关，可以屏蔽以及开启随机给跟随
  void PositionCmdCB(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
  void GoalCB(
      const geometry_msgs::PoseStamped::ConstPtr &msg); // 订阅规划器的目标点

  NodeStatus onStart() override;
  NodeStatus onRunning() override;
  void onHalted() override;
  void CleanValue();

private:
  std::shared_ptr<MavRosConnect> mct;
  std::shared_ptr<ros::NodeHandle> nh;
  const geometry_msgs::PoseStamped *fcu_pose_ptr;
  const mavros_msgs::State *fcu_state_ptr;
  const ros::Publisher *tgt_pose_pub_ptr;
  ros::Publisher planer_goal_pub;
  ros::Subscriber goal_sub; //没有看错，这里的确是订阅规划器的目标点，因为规划器的目标点，也许是别的节点再发。
  ros::Subscriber pos_cmd_sub;
  mavros_msgs::PositionTarget cmd;
  bool is_enable_planner;
  bool has_recv_cmd;
  ros::Time last_time;
  geometry_msgs::PoseStamped goal;
  geometry_msgs::PoseStamped goal_recv;
  
  bool is_stop_planner;  //停止路径规划，退出节点
  bool is_pause_planner;  //暂停路径规划，即把当前节点挂起来
  int goal_src;
  geometry_msgs::Point goal_pos;
  static int last_tra_id;
  bool is_innode;
};
int PlanNode::last_tra_id = 0;
#endif // PLANNODE_H
