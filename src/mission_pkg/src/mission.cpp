#include "mission.h"

Mission::Mission(const std::string &name, const NodeConfig &config)
  : SyncActionNode(name, config)
{
}

PortsList Mission::providedPorts()
{
  ;
}

NodeStatus Mission::tick()
{
  return NodeStatus::SUCCESS;
}
