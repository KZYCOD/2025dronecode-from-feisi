/******************************************************************************
 * @file       pointtgt.h
 * @brief
 *希望飞机能到一个指定位置(如从A点到B点)，可以位置，速度，加速度控制。或者由规划器的输出去控制
 * 但是有一个问题需要考虑，
 *如果目标位置不可达（假如目标点设置在障碍物内部）应该返回失败，
 *
 * @author     戴开世<daivesi@sina.com>
 * @date       2025/03/06
 * @history
 *****************************************************************************/

#ifndef ARRIVEPOINT_H
#define ARRIVEPOINT_H
#include "plugins/common.hpp"
#include "mavros_cnt.h"
#include "geometry_msgs/PoseStamped.h"

using namespace BT;

class ArrivePoint : public SyncActionNode
{
public:
  ArrivePoint(const std::string &name, const NodeConfig &config);

  static PortsList providedPorts();

  NodeStatus tick() override;

private:
  ros::Publisher                    goal_pub;
  std::shared_ptr<MavRosConnect>    mct;
  std::shared_ptr<ros::NodeHandle>  nh;
  const geometry_msgs::PoseStamped *fcu_pose_ptr;
  std::shared_ptr<ros::Publisher>
                              tgt_pub_ptr;  //如果需要使用位置，速度，加速度控制使用这个发布器
  ros::Subscriber             pos_ctrl_sub;
  mavros_msgs::PositionTarget cmd;
};

#endif  // ARRIVEPOINT_H
