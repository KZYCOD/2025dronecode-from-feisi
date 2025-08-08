/******************************************************************************
 * @file       goalyaw.h
 * @brief      这个类很简单就是控制飞机的yaw转向
 *
 * @author     戴开世<daivesi@sina.com>
 * @date       2025/03/31
 * @history
 *****************************************************************************/

#ifndef GOALYAW_H
#define GOALYAW_H

#include "plugins/common.hpp"
#include <eigen3/Eigen/Eigen>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "mavros_cnt.h"

using namespace BT;

class GoalYaw : public SyncActionNode 
{
public:
  GoalYaw(const std::string &name, const NodeConfig &config);
  static PortsList providedPorts();
  NodeStatus       tick() override;

private:
  std::shared_ptr<MavRosConnect>    mct;
  std::shared_ptr<ros::NodeHandle>  nh;
  const geometry_msgs::PoseStamped *fcu_pose_ptr;
  const mavros_msgs::State *        fcu_state_ptr;
  const ros::Publisher *            tgt_pose_pub_ptr;

  mavros_msgs::PositionTarget cmd;
};

#endif  // GOALYAW_H
