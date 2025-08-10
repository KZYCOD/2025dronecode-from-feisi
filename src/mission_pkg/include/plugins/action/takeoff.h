/******************************************************************************
 * @file       takeoff.h
 * @brief      行为树：起飞节点
 * @author     戴开世<daivesi@sina.com>
 * @date       2025/03/06
 * @history
 *****************************************************************************/

#ifndef TAKEOFF_H
#define TAKEOFF_H
#include "plugins/common.hpp"
#include <ros/ros.h>
#include "mavros_cnt.h"
using namespace BT;
class Takeoff : public SyncActionNode
{
public:
  Takeoff(const std::string &name, const NodeConfig &config);

  //定义参数变量获取，在这里是在xml文件内定义，
  static PortsList providedPorts();

  //  static PortsList SimProvidedProts();

  NodeStatus tick() override;

private:
  std::shared_ptr<MavRosConnect>    mct;
  const geometry_msgs::PoseStamped *fcu_pose_ptr;
  const mavros_msgs::State *        fcu_state_ptr;
  ros::ServiceClient *              off_client_ptr;
  ros::ServiceClient *              arm_client_ptr;
  std::shared_ptr<ros::Publisher>   tgt_pose_pub_ptr;
};
#endif  // TAKEOFF_H
