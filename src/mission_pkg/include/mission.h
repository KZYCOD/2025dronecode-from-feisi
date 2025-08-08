#ifndef MISSION_H
#define MISSION_H

#include <plugins/common.hpp>

using namespace BT;

class Mission : public SyncActionNode
{
public:
  Mission(const std::string &name, const NodeConfig &config);

  static PortsList providedPorts();

  NodeStatus tick() override;
};

#endif  // MISSION_H
