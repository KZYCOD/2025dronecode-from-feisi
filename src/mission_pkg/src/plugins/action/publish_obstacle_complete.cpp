#include "plugins/action/publish_obstacle_complete.h"

PublishObstacleComplete::PublishObstacleComplete(const std::string &name, const NodeConfig &config)
    : SyncActionNode(name, config)
{
  complete_pub = nh.advertise<std_msgs::Bool>("/competition/obstacle_complete", 10);
  id_pub = nh.advertise<std_msgs::Int32>("/competition/current_obstacle", 10);
}

PortsList PublishObstacleComplete::providedPorts()
{
  return {
    InputPort<int>("obstacle_id", "完成的障碍编号")
  };
}

NodeStatus PublishObstacleComplete::tick()
{
  auto obstacle_id = getInput<int>("obstacle_id");
  if (!obstacle_id) {
    ROS_ERROR("PublishObstacleComplete: Missing obstacle_id parameter");
    return NodeStatus::FAILURE;
  }

  int id = obstacle_id.value();
  
  // 发布完成状态
  std_msgs::Bool complete_msg;
  complete_msg.data = true;
  complete_pub.publish(complete_msg);
  
  // 发布障碍编号
  std_msgs::Int32 id_msg;
  id_msg.data = id;
  id_pub.publish(id_msg);
  
  ROS_INFO("PublishObstacleComplete: Obstacle %d marked as complete", id);
  
  return NodeStatus::SUCCESS;
}