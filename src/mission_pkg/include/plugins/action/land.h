/******************************************************************************
 * @file       land.h
 * @brief      XXXX Function
 *
 * @author     戴开世<daivesi@sina.com>
 * @date       2025/03/06
 * @history
 *****************************************************************************/

#ifndef LAND_H
#define LAND_H
#include "plugins/common.hpp"
#include <mavros_cnt.h>
#include <ros/ros.h>

using namespace BT;

class Land : public SyncActionNode
{
public:
  Land(const std::string &name, const NodeConfig &config);

  static PortsList providedPorts();
  NodeStatus       tick() override;

private:
  std::shared_ptr<MavRosConnect>    mct;
  const geometry_msgs::PoseStamped *fcu_pose_ptr;
  const mavros_msgs::State *        fcu_state_ptr;
  ros::ServiceClient *              off_client_ptr;
  ros::ServiceClient *              arm_client_ptr;
  const ros::Publisher *            tgt_pose_pub_ptr;
};

#endif  // LAND_H
