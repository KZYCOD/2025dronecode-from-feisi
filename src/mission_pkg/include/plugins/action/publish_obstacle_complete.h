/******************************************************************************
 * @file       publish_obstacle_complete.h
 * @brief      发布障碍完成状态的实用节点
 * @author     AI Assistant
 * @date       2025/01/01
 *****************************************************************************/

#ifndef PUBLISH_OBSTACLE_COMPLETE_H
#define PUBLISH_OBSTACLE_COMPLETE_H

#include "plugins/common.hpp"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

using namespace BT;

class PublishObstacleComplete : public SyncActionNode
{
public:
  PublishObstacleComplete(const std::string &name, const NodeConfig &config);
  
  static PortsList providedPorts();
  NodeStatus tick() override;

private:
  ros::Publisher complete_pub;
  ros::Publisher id_pub;
  ros::NodeHandle nh;
};

#endif // PUBLISH_OBSTACLE_COMPLETE_H